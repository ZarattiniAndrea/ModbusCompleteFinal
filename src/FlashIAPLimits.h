//Assicurarsi che questo file venga incluso una volta sola
#pragma once

#include <Arduino.h>
#include <FlashIAP.h>
#include <FlashIAPBlockDevice.h>

using namespace mbed; 

//Struttura di aiuto nel salvataggio delle proprietà dello storage
struct FlashIAPLimits {
  size_t flash_size; //dimensione della flash
  uint32_t start_address; //indirizzo di inizio
  uint32_t available_size; //memoria disponibile
};

FlashIAPLimits getFlashIAPLimits(){
  // Alignment lambdasù
  //Queste due funzioni lambda sono utilizzate per allineare un valore (val) a un determinato multiplo (definito da size). 
  //In altre parole, queste funzioni ti aiutano a "arrotondare" il valore a un confine allineato al valore specificato.
  auto align_down = [](uint64_t val, uint64_t size) {
    return (((val) / size)) * size;
  };
  auto align_up = [](uint32_t val, uint32_t size) {
    return (((val - 1) / size) + 1) * size;
  };

  size_t flash_size;
  uint32_t flash_start_address;
  uint32_t start_address;
  FlashIAP flash; //istanza di un oggetto della classe FLashIAP. IAP è una tecnica che permette di 
  //programmare la memoria flash in tempo reale. FlashIAP è un calsse fornita dal framework o dalla
  //libreria di supporto.
  auto result = flash.init();
  if(result != 0) return {};

  //trovo il primo settore dopo l'area di testo 
  int sector_size = flash.get_sector_size(FLASHIAP_APP_ROM_END_ADDR);
  start_address = align_up(FLASHIAP_APP_ROM_END_ADDR, sector_size);
  flash_start_address = flash.get_flash_start();
  flash_size = flash.get_flash_size();

  result = flash.deinit();

  int available_size = flash_start_address + flash_size - start_address;
  if (available_size % (sector_size * 2)) {
    available_size = align_down(available_size, sector_size * 2);
  }

  return { flash_size, start_address, available_size };
}