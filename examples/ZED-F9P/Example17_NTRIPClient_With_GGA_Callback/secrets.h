//Your WiFi credentials
const char ssid[] = "yourSSID";
const char password[] = "yourPassword";

//RTK2Go works well and is free
//const char casterHost[] = "rtk2go.com";
//const uint16_t casterPort = 2101;
//const char casterUser[] = "myEmail@test.com"; //User must provide their own email address to use RTK2Go
//const char casterUserPW[] = "";
//const char mountPoint[] = "bldr_SparkFun1"; //The mount point you want to get data from

//Emlid Caster also works well and is free
//const char casterHost[] = "caster.emlid.com";
//const uint16_t casterPort = 2101;
//const char casterUser[] = "u99696"; //User name and pw must be obtained through their web portal
//const char casterUserPW[] = "466zez";
//const char mountPoint[] = "MP1979"; //The mount point you want to get data from

// Skylark (Swift Navigation) is awesome - but requires a subscription:
// https://www.swiftnav.com/skylark
// https://account.swiftnav.com/sign-up
// Use the promo-code ONEMONTHFREE for a free one month access to Skylark on one device
const char casterHost[] = "na.skylark.swiftnav.com"; // na = North Americs L1+L2; eu = Europe L1+L2
const uint16_t casterPort = 2101;
const char casterUser[] = "NTRIPusername+accountSubdomain"; // This is generated when you add a device to your Skylark account
const char casterUserPW[] = "devicePassword";
const char mountPoint[] = "CRS"; // The mount point you want to get data from. Select CRS (Cloud Reference Station) for the ZED-F9x
