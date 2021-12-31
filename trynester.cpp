//
// Created by andyh on 12/28/21.
//

#include "trynester.h"

template <> Stacked<Thrower>* Stacked<Thrower>::tos=nullptr;

template <> Stacked<LocationStack>* Stacked<LocationStack>::tos=nullptr;

 LocationStack::TickSource LocationStack::stamper= nullptr ;

 LocationStack::Reporter LocationStack::reportElapsed= nullptr ;
