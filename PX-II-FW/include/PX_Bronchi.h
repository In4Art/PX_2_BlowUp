#ifndef __PX_BRONCHI__
#define __PX_BRONCHI__

#include "SPI_shiftreg.h"

#define INFLATE_TIME 3500
#define DEFLATE_TIME 5500


typedef enum{
    ERROR = -1,
    EMPTY,
    INFLATING,
    INFLATED,
    DEFLATING
}bronchi_state_t;



class PX_Bronchi{
    public:
        //for use with shift_regs and spare gpio pin
        PX_Bronchi(int8_t num, int8_t n_shift_regs, SPI_shiftreg* shift_regs, uint8_t* spare_pins, uint8_t spare_len);
        //for use with just shift regs, will assume there are enough shift_regs passed
        PX_Bronchi(int8_t num, SPI_shiftreg* shift_regs);

        void inflate(int8_t idx);
        void deflate(int8_t idx);
        bronchi_state_t get_state(int8_t idx); //returns ERROR when trying to access non-existent idx
        void run(void);
        
    private:
        bool _shift_data {false};
        SPI_shiftreg* _shift_regs;
        uint8_t* _extra_pins;
        int8_t _num_bronchi;
        bronchi_state_t* _bronchi_states;
        uint32_t _inflate_t {INFLATE_TIME};
        uint32_t _deflate_t {DEFLATE_TIME};
        uint32_t* _bronchi_times; //pointer to array with times when INFLATE or DEFLATE started
        uint8_t _num_regs {0};

        void set_state(int8_t idx, bronchi_state_t state);

};


#endif
