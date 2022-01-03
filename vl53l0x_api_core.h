/*******************************************************************************
 * Copyright 2021, Andrew Heilveil (github/980f) via massive rework of source:
*   Copyright 2016, STMicroelectronics International N.V.
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
* Neither the name of STMicroelectronics nor the
*      names of its contributors may be used to endorse or promote products
*      derived from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
*   NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
*   IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
*   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
*   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _VL53L0X_API_CORE_H_
#define _VL53L0X_API_CORE_H_

#include "vl53l0x_platform.h" //for Dev_t

namespace VL53L0X {
  //some device independent functions:
  uint8_t encode_vcsel_period(uint8_t vcsel_period_pclks);
  uint8_t decode_vcsel_period(uint8_t vcsel_period_reg);
  /** restoring form of integer square root
   * todo: template arg types, return is half as many bits as given and we could use the extra range on some calculations*/
  uint32_t isqrt(uint32_t num);
  uint32_t quadrature_sum(uint32_t a, uint32_t b);

  /* the time out is 8 bits exponent, 8 bits mantissa special value  */
  uint32_t decode_timeout(uint16_t encoded_timeout);
  uint16_t encode_timeout(uint32_t timeout_macro_clks);
  uint32_t calc_timeout_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

  // the R and S of RMS root mean square
  template<unsigned whole, unsigned fract>
  static auto quadrature_sum(FixPoint<whole, fract> a, FixPoint<whole, fract> b) -> typename decltype(a)::RawType {
    return quadrature_sum(a.raw, b.raw);
  }

  class Core : public Dev_t {
  public:
    Core(Arg &&args) : Dev_t(std::forward<Arg>(args)) {
      //do nothing here so that we can statically construct
    }

/**
 * @brief Gets the (on/off) state of a requested sequence step.
 *
 * @par Function Description
 * This function retrieves the state of a requested sequence step, i.e. on/off.
 *
 * @note This function Accesses the device
 *
 * @param   SequenceStepId         Sequence step identifier.
 */
    bool GetSequenceStepEnable(SequenceStepId StepId); // GetSequenceStepEnable
/**
 * @brief Reads the Product Revision for a for given Device
 * This function can be used to distinguish cut1.0 from cut1.1.
 *
 * @note This function Access to the device
 *
 * @return  major/minor aka "cut"
 */
    SemverLite GetProductRevision();

    /** VCSELPulsePeriodPCLK */
    uint8_t get_vcsel_pulse_period(VcselPeriod VcselPeriodType);

    /**
     * @note: refactoring dropped a call to  perform_phase_calibration
     * ... callers of this method should call that.
     * */
    bool set_vcsel_pulse_period(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK);

    enum InfoGroup {
      SpadStuff = Bitter(0)
      , IDStuff = Bitter(1)
      , PartUidEtc = Bitter(2)
      , ALL = Mask<2, 0>::places
    };

    bool get_info_from_device(uint8_t infoGroupBits);

    uint32_t get_sequence_step_timeout(SequenceStepId SequenceStepId);

    bool set_sequence_step_timeout(SequenceStepId SequenceStepId, uint32_t TimeOutMicroSecs);

    uint32_t get_measurement_timing_budget_micro_seconds();

    bool set_measurement_timing_budget_micro_seconds(uint32_t MeasurementTimingBudgetMicroSeconds);

    /** @returns number of items successfully sent. you can check against size of your table.
     * */
    unsigned int load_tuning_settings(const uint8_t *pTuningSettingBuffer);

    /** formerly returned a value that was then written to a member of the RMD parameter, whose value is never queried by the driver.
     * @returns sigma estimate, udpates RangeDMaxMillimeter as a side effect of the computation.
     * */
    FixPoint1616_t calc_sigma_estimate(RangingMeasurementData_t &pRangingMeasurementData);

    MegaCps get_total_xtalk_rate(const RangingMeasurementData_t &pRangingMeasurementData);

    MegaCps get_total_signal_rate(const RangingMeasurementData_t &pRangingMeasurementData);

    /** RMD not const due to setting of darkMM */
    RangeStatus get_pal_range_status(uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate, uint16_t EffectiveSpadRtnCount, RangingMeasurementData_t &pRangingMeasurementData);

    /**
 * @brief  Get specific limit check enable state
 *
 * @par Function Description
 * This function get the enable state of a specific limit check.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function Access to the device
 *
 * @param   LimitCheckId                  Limit Check ID
 * @return  whether the selected limit is enabled
 */
    bool GetLimitCheckEnable(CheckEnable LimitCheckId);

/**
 * @brief  Get a specific limit check value
 *
 * @par Function Description
 * This function get a specific limit check value from device then it updates
 * internal values and check enables.
 * The limit check is identified with the LimitCheckId.
 *
 * @note This function Access to the device
 *
 * @param   LimitCheckId                  Limit Check ID
 * @return  Limit check Value for a given LimitCheckId
 */
    Cps16 GetLimitCheckValue(CheckEnable LimitCheckId);

    /**
     * @returns the @param LimitCheckId 's enable and value.
     * */
    LimitTuple GetLimitCheck(CheckEnable LimitCheckId) {
      LimitTuple collector;
      collector.enable = GetLimitCheckEnable(LimitCheckId);
      collector.value = GetLimitCheckValue(LimitCheckId);
      return collector;
    }

    /**
     * An instance of this class ensures that a matching operation such as clearing a pause bit in the device occurs even when a procedure bails out early.
     * You may see otherwise extraneous {} surround code to control the timing of that second operation.
     *
     * Since we are using longjmp instead of exceptions for hopeless errors 'throw' leaves the device in an unknown state.
     * Since the (psuedo) exceptions are (but for a few that we may remove) due to communications failure doing a complete restart on any error makes sense.
     * */
    class SysPopper {
    protected:
      Physical &comm;
      uint8_t index;
      uint8_t pop;
    public:
      SysPopper(Physical &comm, uint8_t index, uint8_t push, uint8_t pop) : comm(comm), index(index), pop(pop) {
        comm.WrByte(index, push);
      }

      ~SysPopper() {
        comm.WrByte(index, pop);
      }
    };

    /** call this to ensure a restore operation occurs when the object goes out of scope. */
    SysPopper push(uint8_t index, uint8_t push, uint8_t pop) {
      return {comm, index, push, pop};
    }

    /** writes an object after first writing 1 to 0xFF , writing a 0 to FF when done */
    template<typename Intish> void FFwrap(RegSystem index, Intish value) {
      push(Private_Pager, 0x01, 0x00);
      comm.WriteMulti(index, reinterpret_cast<uint8_t *>(&value), sizeof(Intish));
    }

    template<typename Intish> Intish FFread(RegSystem index) {
      Intish value;
      push(Private_Pager, 0x01, 0x00);
      comm.ReadMulti(index, reinterpret_cast<uint8_t *>(&value), sizeof(Intish));
      return value;
    }

    /** sometimes a matched pair of operations needs each access wrapped with 0xFF accesses*/
    class FFPopper {
      Physical &comm;
      uint8_t index;
      uint8_t pop;
    public:
      FFPopper(Physical &comm, uint8_t index, uint8_t push, uint8_t pop) : comm(comm), index(index), pop(pop) {
        auto popper = SysPopper(comm, Private_Pager, 0x01, 0x00);
        comm.WrByte(index, push);
      }

      ~FFPopper() {
        //NB: this won't happen when an Error is thrown
        auto popper = SysPopper(comm, Private_Pager, 0x01, 0x00);
        comm.WrByte(index, pop);
      }
    };

    /** call this to ensure a restore operation occurs when the object goes out of scope. */
    FFPopper FFpush(uint8_t index, uint8_t push, uint8_t pop) {
      return {comm, index, push, pop};
    }

    /** Yet Another Popper
     * undocumented magic access to a write only bit
     * comm.WrByte(0xFF, 0x06);
      comm.UpdateBit(0x83, 2, true to start, false when done);
     * */
    class YAPopper {
      Physical &comm;
    public:
      YAPopper(Physical &comm) : comm(comm) {
        comm.WrByte(Private_Pager, 0x06);
        comm.UpdateBit(0x83, 2, true);
      }

      ~YAPopper() {
        comm.WrByte(Private_Pager, 0x06);
        comm.UpdateBit(0x83, 2, false);
      }
    };

    class MagicDuo : SysPopper {
      SysPopper inner;
    public:
      MagicDuo(Physical &comm) : SysPopper(comm, Private_Pager, 0x01, 0x00), inner(comm, 0x00, 0x00, 0x01) {
      }

      ~MagicDuo() = default;
    };

    class MagicTrio {
      SysPopper eighty;
      MagicDuo duo;
    public:
      MagicTrio(Physical &comm) : eighty(comm, 0x80, 0x01, 0x00), duo(comm) {
      }

      ~MagicTrio() = default;
    };

    /** RAII widget that surrounds some fetches. */
    MagicTrio magicWrapper() {
      return {comm};
    }

    /** creating one of these saves sequence config, sets it, and on destruction restores what was saved. */
    class SeqConfigStacker {
      Core &core;
      uint8_t mycache;
      bool restore_it;
    public:
      SeqConfigStacker(Core &core, bool restore_config, uint8_t settit) : core(core) {
        mycache = core.get_SequenceConfig();
        core.set_SequenceConfig(settit, false);//todo:0 probably should set andCache true, else could use the dev cached one instead of our own.
        restore_it = restore_config; //maydo: don't restore if we failed to set.
      }

      ~SeqConfigStacker() {
        if (restore_it) {
          /* restore the previous Sequence Config */
          core.set_SequenceConfig(mycache, true);
        }
      }
    };

  protected:  //common code fragments and what were file static but didn't actually have the 'static' like they should have.

/** gets value from device.
 * to do this as a simple return we would have to explicitly put the type in <> and that would be painful without a macro
 * */
    template<typename Scalar> void fetch(Scalar &item, RegSystem reg) {
      comm.Read<Scalar>(reg, item);
    }

    /** gets value from device, @returns whether it worked OK.*/
    template<unsigned whole, unsigned fract> void fetch(FixPoint<whole, fract> &item, RegSystem reg) {
      fetch<typename FixPoint<whole, fract>::RawType>(reg, item.raw);
    }

    void setValidPhase(uint8_t high, uint8_t low = 8);

    void setPhasecalLimit(uint8_t value);

  protected: //some of this functionality was in api.cpp but used by core.cpp
    /** spin until a particular byte is non-zero.
     * @returns whether that occurred, false on a timeout set by @param trials loop count */
    bool device_read_strobe(unsigned trials = VL53L0X_DEFAULT_MAX_LOOP);

    /** read and return up to 4 bytes from 0x90 after selecting which at 0x94
     * NB: source for templates can be in the CPP if not used outside that module :) */
    template<typename Int> Int packed90(uint8_t which);

    /** pulls a 24:8 from a 32:32 */
    uint32_t middleof64(unsigned int which);

    /** reads and @returns the sequence steps and broken out into a struct of bools. */
    SchedulerSequenceSteps_t get_sequence_step_enables();

    /** @returns an interesting computation that someone should document */
    uint32_t calc_dmax(MegaCps totalSignalRate_mcps, MegaCps totalCorrSignalRate_mcps, FixPoint1616_t pwMult, uint32_t sigmaEstimateP1, FixPoint1616_t sigmaEstimateP2, uint32_t peakVcselDuration_us);
/**
 * @brief Enable/Disable Cross talk compensation feature
 *
 * old code and documentation acted like the enable was implemented via setting the value to zero, but now there is a definite device boolean as well as the threshold.
 * @a SetXTalkCompensationRateMegaCps().
 *
 * @param   XTalkCompensationEnable   Cross talk compensation to be set 0=disabled else = enabled
 */
    void SetXTalkCompensationEnable(bool XTalkCompensationEnable);
/**
 * @brief Get Cross talk compensation rate
 *
 * @note This function is not Implemented. (Says who!) //BUG: stale documentation
 * Enable/Disable Cross Talk by set to zero the Cross Talk value by
 * using @a SetXTalkCompensationRateMegaCps().
 *
 * @return  whether xtalk compensation is enabled
 */
    bool GetXTalkCompensationEnable();

/**
 * @brief Set Cross talk compensation rate
 *
 * @par Function Description
 * Set Cross talk compensation rate.
 *
 * @note This function Access to the device
 *
 * @param   XTalkCompensationRateMegaCps   Compensation rate in Mega counts per second (16.16 fix point) see datasheet for details
 * @return Success
 */
    bool SetXTalkCompensationRateMegaCps(MegaCps XTalkCompensationRateMegaCps);

/**
 * @brief Get Cross talk compensation rate
 *
 * @par Function Description
 * Get Cross talk compensation rate.
 *
 * @note This function Access to the device
 *
 * @return  xtalk comp rate in megacps
 */
    MegaCps GetXTalkCompensationRateMegaCps();

    void set_ref_spad_map(SpadArray &refSpadArray);//writes to device
    void get_ref_spad_map(SpadArray &refSpadArray);//read from device
    /** @returns the sequence config byte reading it from the device */
    uint8_t get_SequenceConfig();
    /** sets the sequence config byte, and if @param andCache is true copies the value to the DeviceParameters place where we often trust has the value */
    void set_SequenceConfig(uint8_t packed, bool andCache = true);

    void load_compact(const DeviceByte *bunch, unsigned quantity);

    template<typename Scalar>
    void send(const DeviceValue<Scalar> item) {
      comm.Write(item.index, item.pattern, sizeof(Scalar) > 1);
    }

    bool oneTuning(const uint8_t *&pTuningSettingBuffer);
  };
}//end namespace
#endif /* _VL53L0X_API_CORE_H_ */
