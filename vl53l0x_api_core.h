/*******************************************************************************
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
#include "vl53l0x_fixpoint.h"

namespace VL53L0X {
  //some device independent functions:
  void reverse_bytes(uint8_t *data, uint32_t size);
  uint8_t encode_vcsel_period(uint8_t vcsel_period_pclks);
  uint8_t decode_vcsel_period(uint8_t vcsel_period_reg);
  /** restoring form of integer square root
   * todo: template arg types, return is half as many bits as given and we could use the extra range on some calculations*/
  uint32_t isqrt(uint32_t num);
  uint32_t quadrature_sum(uint32_t a, uint32_t b);
  uint32_t decode_timeout(uint16_t encoded_timeout);
  uint16_t encode_timeout(uint32_t timeout_macro_clks);
  uint32_t calc_timeout_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

  template<unsigned whole, unsigned fract>
  static auto quadrature_sum(FixPoint<whole, fract> a, FixPoint<whole, fract> b) -> typename decltype(a)::RawType {
    //todo: debate whether the guard in the non-class quadrature_sum should apply
//      if (a > 65535 || b > 65535) {
//        return 65535;
//      }
    return isqrt(a.squared() + b.squared());
  }

  class Core : public Dev_t {
  public:
    Core(Arg &&args) : Dev_t(std::forward<Arg>(args)) {
      //do nothing here so that we can statically construct
    }

    struct FakeTrace {
      const char *location="";
      unsigned line=0;

    } theTrace;
    void throwException(const char *location,unsigned line,Error error){
      theTrace={location,line};
      longjmp(comm.wirer.ComException, error); // NOLINT(cert-err52-cpp)   exceptions not allowed on our platform
    }

    bool GetSequenceStepEnable(SequenceStepId StepId); // GetSequenceStepEnable

    SemverLite GetProductRevision();

    /** VCSELPulsePeriodPCLK */
    uint8_t get_vcsel_pulse_period(VcselPeriod VcselPeriodType);

    /**
     * todo: refactoring dropped a call to  perform_phase_calibration(&PhaseCalInt, false, true); which is in API, not CORE.
     * ... callers of this method should call that.
     * */
    bool set_vcsel_pulse_period(VcselPeriod VcselPeriodType, uint8_t VCSELPulsePeriodPCLK);

    enum InfoGroup {
      SpadStuff = Bitter(0)
      , IDStuff = Bitter(1)
      , PartUidEtc = Bitter(2)
      , ALL = Mask<2,0>::places
    };

    bool get_info_from_device(uint8_t infoGroupBits);

    uint32_t get_sequence_step_timeout(SequenceStepId SequenceStepId);

    bool set_sequence_step_timeout(SequenceStepId SequenceStepId, uint32_t TimeOutMicroSecs);

    uint32_t get_measurement_timing_budget_micro_seconds();

    bool set_measurement_timing_budget_micro_seconds(uint32_t MeasurementTimingBudgetMicroSeconds);

    /** @returns inverse of index of byte causing early exit. 0 if no problems.
     * a nonzero return should probably be responded to by a restart of the interface.
     * */
    unsigned int load_tuning_settings(const uint8_t *pTuningSettingBuffer);

    /** ?secondary return via reference to dmm as most if it is computed as a side effect of primary return.
     * */
    FixPoint1616_t calc_sigma_estimate(const RangingMeasurementData_t &pRangingMeasurementData, uint32_t &dmm);

    FixPoint1616_t get_total_xtalk_rate(const RangingMeasurementData_t &pRangingMeasurementData);

    FixPoint1616_t get_total_signal_rate(const RangingMeasurementData_t &pRangingMeasurementData);

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
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   pLimitCheckEnable             Pointer to the check limit enable
 * value.
 *  if 1 the check limit
 *        corresponding to LimitCheckId is Enabled
 *  if 0 the check limit
 *        corresponding to LimitCheckId is disabled
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned
 *  when LimitCheckId value is out of range.
 * @return  "Other error code"            See ::Error
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
 * @param   Dev                           Device Handle
 * @param   LimitCheckId                  Limit Check ID
 *  (0<= LimitCheckId < GetNumberOfLimitCheck() ).
 * @param   pLimitCheckValue              Pointer to Limit
 *  check Value for a given LimitCheckId.
 * @return  ERROR_NONE             Success
 * @return  ERROR_INVALID_PARAMS   This error is returned
 *  when LimitCheckId value is out of range.
 * @return  "Other error code"            See ::Error
 */
    FixPoint<9, 7> GetLimitCheckValue(CheckEnable LimitCheckId);

    /** devolved into same code as FrameEnder, removed it in favor of this as being in a better file  */
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

    /** writes an object after first writing 1 to 0xFF , writing a 0 to FF when done, returnin accumulated error */
    template<typename Intish> void FFwrap(RegSystem index, Intish value) {
      push(0xFF, 0x01, 0x00);
      comm.WriteMulti(index, reinterpret_cast<uint8_t *>(&value), sizeof(Intish));
    }

    template<typename Intish> Intish FFread(RegSystem index) {
      Intish value;
      push(0xFF, 0x01, 0x00);
      comm.ReadMulti(index, reinterpret_cast<uint8_t *>(&value), sizeof(Intish));
      return value;
    }

    class FFPopper  {
      Physical &comm;
      uint8_t index;
      uint8_t pop;
    public:
      FFPopper(Physical &comm, uint8_t index, uint8_t push, uint8_t pop) : comm(comm), index(index), pop(pop) {
        auto popper = SysPopper(comm, 0xFF, 0x01, 0x00);
        comm.WrByte(index, push);
      }

      ~FFPopper() {
        //todo: not try if push fails?
        auto popper = SysPopper(comm, 0xFF, 0x01, 0x00);
        comm.WrByte(index, pop);
      }
    };

    /** call this to ensure a restore operation occurs when the object goes out of scope. */
    FFPopper FFpush(uint8_t index, uint8_t push, uint8_t pop) {
      return {comm, index, push, pop};
    }

    /**
     * comm.WrByte(0xFF, 0x06);
      comm.UpdateBit(0x83, 2, true to start, false when done);
     * */
    class YAPopper{
      Physical &comm;
    public:
      YAPopper(Physical &comm):comm(comm){
        comm.WrByte(0xFF, 0x06);
        comm.UpdateBit(0x83, 2, true);
      }
      ~YAPopper(){
        comm.WrByte(0xFF, 0x06);
        comm.UpdateBit(0x83, 2, false);
      }
    };


    class MagicDuo {
      Physical &comm; //not a base as eventually we will pass in a reference to a baser class
    public:
      MagicDuo(Physical &comm) : comm(comm) {
        comm.WrByte(0xFF, 0x01);
        comm.WrByte(0x00, 0x00);
      }

      ~MagicDuo() {
        comm.WrByte(0x00, 0x01);
        comm.WrByte(0xFF, 0x00);
      }
    };

    class MagicTrio {
      SysPopper eighty;
      MagicDuo duo;
    public:
      MagicTrio(Physical &comm) :eighty(comm,0x80, 0x01,0x00), duo(comm) {
      }

      ~MagicTrio() {  };
    };

    /** RAII widget that surrounds some fetches. */
    MagicTrio magicWrapper() {
      return {comm};
    }

    /** creating one of these saves sequence config, sets it, and on destruction restores what was saved.
     * Inheriting rather than containing ErrorAccumulator so that we can share some macros. */
    class SeqConfigStacker  {
      Core &core;
      uint8_t mycache;
      bool restore_it;
    public:
      SeqConfigStacker(Core &core, bool restore_config, uint8_t settit) : core(core) {
        mycache=core.get_SequenceConfig();
        core.set_SequenceConfig(settit,false);//todo: probably should set andChace true
        restore_it = restore_config; //maydo: don't restore if we failed to set.
      }

      ~SeqConfigStacker() {
        if (restore_it) {
          /* restore the previous Sequence Config */
          core.set_SequenceConfig(mycache, true);
        }
      }
    };

  protected:  //common code fragments or what were file static but didn't actually have the 'static' like they should have.



/** gets value from device, @returns whether it worked OK.*/
    template<typename Scalar> void fetch(Scalar &item, RegSystem reg) {
      comm.Read<Scalar>(reg, item);
    }

    /** gets value from device, @returns whether it worked OK.*/
    template<unsigned whole, unsigned fract> void fetch(FixPoint<whole, fract> &item, RegSystem reg) {
      fetch<typename FixPoint<whole, fract>::RawType>(reg, item.raw);
    }

    void setValidPhase(uint8_t high, uint8_t low=8);

    void setPhasecalLimit(uint8_t value);

  protected: //some of this functionality was in api.cpp but used by core.cpp
    bool device_read_strobe();
    /** read up to 4 bytes from 0x90 after selecting which at 0x94
     * source for templates can be in the CPP if not used outside that module :) */
    template<typename Int> Int packed90(uint8_t which);

    /** pulls a 24:8 from a 32:32 */
    uint32_t middleof64(unsigned int which);

    /** reads the sequence steps and breaks them out into a struct of bools. */
    SchedulerSequenceSteps_t get_sequence_step_enables();

    /** @returns an interesting computation that someone should document */
    uint32_t calc_dmax(FixPoint1616_t totalSignalRate_mcps, FixPoint1616_t totalCorrSignalRate_mcps, FixPoint1616_t pwMult, uint32_t sigmaEstimateP1, FixPoint1616_t sigmaEstimateP2, uint32_t peakVcselDuration_us);

    void SetXTalkCompensationEnable(bool XTalkCompensationEnable);

    void set_ref_spad_map(SpadArray &refSpadArray);//writes to device
    void get_ref_spad_map(SpadArray &refSpadArray);//read from device
    /** @returns the sequence config byte reading it from the device */
    uint8_t get_SequenceConfig();
    /** sets the sequence config byte, and if @param andCache is true copies the value to the DeviceParameters place where we often trust has the value */
    void set_SequenceConfig(uint8_t packed, bool andCache);
  };
}//end namespace
#endif /* _VL53L0X_API_CORE_H_ */
