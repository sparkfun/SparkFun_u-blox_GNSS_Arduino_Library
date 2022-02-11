// You can set the information below after signing up with the u-blox Thingstream portal 
// and adding a new New PointPerfect Thing
// https://portal.thingstream.io/app/location-services/things
// In the new PointPerfect Thing, you go to the credentials tab and copy and paste the IP Dynamic Keys here.
//
// The keys are valid from a particular GPS Week Number and Time of Week.
// Looking at the credentials tab, the current key expires 23:59 Feb 11th 2022.
// This means the next key is valid _from_ Midnight Feb 12th 2022.
// That is GPS Week 2196. The GPS Time of Week in seconds is 518418.
// Working backwards, the current key became valid exactly 4 weeks earlier (Midnight Jan 15th 2022).
//
// See: https://www.labsat.co.uk/index.php/en/gps-time-calculator
//
// The keys are given as: 32 hexadecimal digits = 128 bits = 16 Bytes

const uint8_t currentKeyLengthBytes =   16; 
const uint8_t currentDynamicKey[] =     "f742bd6b7248043177dd649141d8fb0b";
const uint16_t currentKeyGPSWeek =      2192;
const uint32_t currentKeyGPSToW =       518418;

const uint8_t nextKeyLengthBytes =      16; 
const uint8_t nextDynamicKey[] =        "8206........................29f4";
const uint16_t nextKeyGPSWeek =         2196;
const uint32_t nextKeyGPSToW =          518418;
