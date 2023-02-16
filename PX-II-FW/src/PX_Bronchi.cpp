#include "PX_Bronchi.h"


PX_Bronchi::PX_Bronchi(int8_t num, int8_t num_regs, SPI_shiftreg* shift_regs, uint8_t* spare_pins, uint8_t spare_len)
{
   _shift_regs  = shift_regs;
   _num_regs = num_regs;

   if((num * 2) > (_num_regs * 8)){
       //uint8_t spare_pins[] = {D0, D1, 9, 10};
       uint8_t nextra = (num * 2) - (_num_regs * 8);
       if(nextra > spare_len){
           Serial.println("PX_Bronchi ERROR: NOT ENOUGH GPIO_PINS AVAILABLE");
       }else{
           _extra_pins = new uint8_t[nextra];
           for(uint8_t i = 0; i < nextra; i++){
               *(_extra_pins + i) = *(spare_pins + i);
               pinMode(*(_extra_pins + i), OUTPUT);
           }
       }
   }

   _num_bronchi = num;
   _bronchi_states = new bronchi_state_t[_num_bronchi];
   _bronchi_times = new uint32_t[_num_bronchi];
   for(uint8_t i = 0; i < _num_bronchi; i++){
       *(_bronchi_states + i) = INFLATED;
       *(_bronchi_times + i) = millis();
   }

   
   
  // _shift_regs->clear_all();
}

PX_Bronchi::PX_Bronchi(int8_t num, SPI_shiftreg* shift_regs)
{
    _shift_regs = shift_regs;
    _num_bronchi = num;
    _bronchi_states = new bronchi_state_t[_num_bronchi];
    _bronchi_times = new uint32_t[_num_bronchi];
   for(uint8_t i = 0; i < _num_bronchi; i++){
       *(_bronchi_states + i) = EMPTY;
       *(_bronchi_times + i) = millis();
   }
   //_shift_regs->clear_all();
}

void PX_Bronchi::inflate(int8_t idx)
{   
    bronchi_state_t state = get_state(idx);
    //eject from function if inflate is an invalid command
    if(state == ERROR || state == INFLATING || state == INFLATED || state == DEFLATING){
        return;
    }else{
        
        //record time and open inflate valve
        *(_bronchi_times + idx) = millis();
        *(_bronchi_states + idx) = INFLATING;

        uint8_t valve_idx = idx * 2;

        if (valve_idx < _num_regs * 8)
        {
            _shift_regs->set_data_bit(valve_idx, 1); //0, 2, 4, 6 etc
            _shift_data = true;                      //we set this so run() can shift data
        }
        else
        {
            uint8_t xtr_idx = valve_idx % (_num_regs * 8);
            digitalWrite(*(_extra_pins + xtr_idx), HIGH);
        }
    }

}

void PX_Bronchi::deflate(int8_t idx)
{
    bronchi_state_t state = get_state(idx);
    //eject from function if inflate is an invalid command
    if(state == ERROR || state == INFLATING || state == DEFLATING || state == EMPTY){
        return;
    }else{
       
        *(_bronchi_times + idx) = millis();
        *(_bronchi_states + idx) = DEFLATING;
        uint8_t valve_idx = idx * 2 + 1;
        if(valve_idx < _num_regs * 8)
        {
        _shift_regs->set_data_bit(idx * 2 + 1, 1 ); //1, 3, 5, 7 etc
        _shift_data = true; //we set this so run() can shift data
        }
        else
        {
            uint8_t xtr_idx = valve_idx % (_num_regs * 8);
            digitalWrite(*(_extra_pins + xtr_idx), HIGH);
        }
    }

}


bronchi_state_t PX_Bronchi::get_state(int8_t idx){
    if(idx < 0 && idx > _num_bronchi){
        return ERROR;
    }else{
        return *(_bronchi_states + idx);
    }
}

void PX_Bronchi::set_state(int8_t idx, bronchi_state_t state)
{
    if(idx >= 0 && idx < _num_bronchi){
        *(_bronchi_states + idx) = state;
    }

}

void PX_Bronchi::run(void)
{
    //do we need to  close any valves?
    bronchi_state_t state = ERROR; //init to error
    for(int8_t i = 0; i < _num_bronchi; i++){
        state = get_state(i);
        if(state == INFLATING){
            if(millis() - *(_bronchi_times + i) >= _inflate_t){

                uint8_t idx = i * 2;
                if(idx < _num_regs * 8)
                {
                    _shift_regs->set_data_bit(i * 2, 0);
                    _shift_data = true;
                }
                else
                {
                     uint8_t xtr_idx = idx % (_num_regs * 8);
                     digitalWrite(*(_extra_pins + xtr_idx), LOW);
                }
                set_state(i, INFLATED);
               
                
            }
        }else if(state == DEFLATING){
            if(millis() - *(_bronchi_times + i) >= _deflate_t){
                uint8_t idx = i * 2 + 1;
                if(idx < _num_regs * 8)
                {
                _shift_regs->set_data_bit(i * 2 + 1 , 0);
                _shift_data = true;
                }
                else
                {
                    uint8_t xtr_idx = idx % (_num_regs * 8);
                    digitalWrite(*(_extra_pins + xtr_idx), LOW);
                }
                set_state(i, EMPTY);
                
            }
        }
    }

    if(_shift_data){
        
        _shift_regs->shift_data();
        _shift_data = false;
    }

}