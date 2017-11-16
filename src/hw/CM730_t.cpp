/**
 *  @autor tekatod
 *  @date 9/20/17
 */
#include "hw/FSR_t.h"
#include "hw/CM730_t.h"

using namespace drwn;


#define ID                  (2)
#define LENGTH              (3)
#define INSTRUCTION         (4)
#define ERRBIT              (4)
#define PARAMETER           (5)
#define DEFAULT_BAUDNUMBER  (1)

#define INST_PING           (1)
#define INST_READ           (2)
#define INST_WRITE          (3)
#define INST_REG_WRITE      (4)
#define INST_ACTION         (5)
#define INST_RESET          (6)
#define INST_SYNC_WRITE     (131)   // 0x83
#define INST_BULK_READ      (146)   // 0x92


bulk_read_data_t::bulk_read_data_t()
        :
        start_address(0),
        length(0),
        error(-1) {
    for (int i = 0; i < MX28_t::MAXNUM_ADDRESS; i++)
        table[i] = 0;
}


int bulk_read_data_t::read_byte(int address) {
    if (address >= start_address && address < (start_address + length))
        return (int) table[address];

    return 0;
}


int bulk_read_data_t::read_word(int address) {
    if (address >= start_address && address < (start_address + length))
        return CM730_t::make_word(table[address], table[address + 1]);

    return 0;
}

int CM730_t::make_word(int lowbyte, int highbyte) {
    unsigned short word;

    word = highbyte;
    word = word << 8;
    word = word + lowbyte;

    return (int) word;
}


int CM730_t::get_low_byte(int word) {
    unsigned short temp;
    temp = word & 0xff;
    return (int) temp;
}


int CM730_t::get_high_byte(int word) {
    unsigned short temp;
    temp = word & 0xff00;
    return (int) (temp >> 8);
}


int CM730_t::make_color(int red, int green, int blue) {
    int r = red & 0xFF;
    int g = green & 0xFF;
    int b = blue & 0xFF;

    return (int) (((b >> 3) << 10) | ((g >> 3) << 5) | (r >> 3));
}
