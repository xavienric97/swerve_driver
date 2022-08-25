#pragma once

//Register addresses
constexpr static uint32_t CANBase = 0x40006400;

constexpr static uint32_t mcr = CANBase + 0x000;  // master cntrl
constexpr static uint32_t msr = CANBase + 0x004;  // rx status
constexpr static uint32_t tsr = CANBase + 0x008;  // tx status
constexpr static uint32_t rf0r = CANBase + 0x00C; // rx fifo 0 info reg

constexpr static uint32_t ier = CANBase + 0x014; // interrupt enable

constexpr static uint32_t btr = CANBase + 0x01C; // bit timing and rate

constexpr static uint32_t ti0r = CANBase + 0x180;  // tx mailbox id
constexpr static uint32_t tdt0r = CANBase + 0x184; // tx data len and time stamp
constexpr static uint32_t tdl0r = CANBase + 0x188; // tx mailbox data[3:0]
constexpr static uint32_t tdh0r = CANBase + 0x18C; // tx mailbox data[7:4]

constexpr static uint32_t ri0r = CANBase + 0x1B0;  // rx fifo id reg
constexpr static uint32_t rdt0r = CANBase + 0x1B4; // fifo data len and time stamp
constexpr static uint32_t rdl0r = CANBase + 0x1B8; // rx fifo data low
constexpr static uint32_t rdh0r = CANBase + 0x1BC; // rx fifo data high

constexpr static uint32_t fmr = CANBase + 0x200;   // filter master reg
constexpr static uint32_t fm1r = CANBase + 0x204;  // filter mode reg
constexpr static uint32_t fs1r = CANBase + 0x20C;  // filter scale reg, 16/32 bits
constexpr static uint32_t ffa1r = CANBase + 0x214; //filter FIFO assignment
constexpr static uint32_t fa1r = CANBase + 0x21C;  // filter activation reg
constexpr static uint32_t fr1 = CANBase + 0x240;   // id/mask acceptance reg1
constexpr static uint32_t fr2 = CANBase + 0x244;   // id/mask acceptance reg2

constexpr static uint32_t scsBase = 0xE000E000UL;        // System Control Space Base Address
constexpr static uint32_t nvicBase = scsBase + 0x0100UL; // NVIC Base Address
constexpr static uint32_t iser = nvicBase + 0x000;       //  NVIC interrupt set (enable)
constexpr static uint32_t icer = nvicBase + 0x080;       // NVIC interrupt clear (disable)

constexpr static uint32_t scbBase = scsBase + 0x0D00UL;
constexpr static uint32_t vtor = scbBase + 0x008;

// GPIO/AFIO Regs
constexpr static uint32_t afioBase = 0x40010000UL;
constexpr static uint32_t mapr = afioBase + 0x004; // alternate pin function mapping

constexpr static uint32_t gpioABase = 0x40010800UL; // port A
constexpr static uint32_t crhA = gpioABase + 0x004; // cntrl reg for port b
constexpr static uint32_t odrA = gpioABase + 0x00c; // output data reg

constexpr static uint32_t gpioBBase = gpioABase + 0x400; // port B
constexpr static uint32_t crhB = gpioBBase + 0x004;      // cntrl reg for port b
constexpr static uint32_t odrB = gpioBBase + 0x00c;      // output data reg

// Clock
constexpr static uint32_t rcc = 0x40021000UL;
constexpr static uint32_t rccBase = 0x40021000UL;
constexpr static uint32_t apb1enr = rccBase + 0x01c;
constexpr static uint32_t apb2enr = rccBase + 0x018;