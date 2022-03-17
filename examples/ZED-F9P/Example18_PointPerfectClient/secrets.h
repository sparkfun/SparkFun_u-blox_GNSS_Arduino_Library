//Your WiFi credentials
const char ssid[] = "<YOUR SSID>";
const char password[] =  "<YOUR PASSWORD>";

// Below infomation you can set after signing up with u-blox Thingstream portal 
// and after add a new New PointPerfect Thing
// https://portal.thingstream.io/app/location-services/things
// in the new PointPerfect Thing you go to the credentials page and copy paste the values and certificate into this.  

// <Your PointPerfect Thing> -> Credentials -> Hostname
const char AWS_IOT_ENDPOINT[]       = "pp.services.u-blox.com";
const unsigned short AWS_IOT_PORT   = 8883;
// <Your PointPerfect Thing> -> Credentials -> IP key distribution topic
const char MQTT_TOPIC_KEY[]        = "/pp/ubx/0236/ip"; // This topic provides the IP only dynamic keys in UBX format
//const char MQTT_TOPIC_KEY[]        = "/pp/ubx/0236/Lb"; // This topic provides the L-Band + IP dynamic keys in UBX format
// <Your PointPerfect Thing> -> Credentials -> IP correction topic for EU/US region
const char MQTT_TOPIC_SPARTN[]     = "/pp/ip/us"; // This topic provides the SPARTN corrections for IP only: choice of {eu, us}
//const char MQTT_TOPIC_SPARTN[]     = "/pp/Lb/us"; // This topic provides the SPARTN corrections for L-Band and L-Band + IP: choice of {eu, us}
// <Your PointPerfect Thing> -> Credentials -> AssistNow (MGA) topic
const char MQTT_TOPIC_ASSISTNOW[]     = "/pp/ubx/mga";

// <Your PointPerfect Thing> -> Credentials -> Client Id
static const char MQTT_CLIENT_ID[] = "<ADD YOUR CLIENT ID HERE>";

// <Your PointPerfect Thing> -> Credentials -> Amazon Root Certificate
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
<ADD YOUR CERTICICATE HERE>
-----END CERTIFICATE-----
)EOF";

// <Your PointPerfect Thing> -> Credentials -> Client Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
<ADD YOUR CERTICICATE HERE>
-----END CERTIFICATE-----
)KEY";

// Get this from Thingstream Portal 
// <Your PointPerfect Thing> -> Credentials -> Client Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
<ADD YOUR KEY HERE>
-----END RSA PRIVATE KEY-----
)KEY";
