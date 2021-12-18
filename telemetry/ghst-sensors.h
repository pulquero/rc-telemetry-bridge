#define GHST_DL_OPENTX_SYNC             0x20
#define GHST_DL_LINK_STAT               0x21
#define GHST_DL_VTX_STAT                0x22
#define GHST_DL_PACK_STAT               0x23
#define GHST_DL_MENU_DESC               0x24
#define GHST_DL_GPS_PRIMARY             0x25
#define GHST_DL_GPS_SECONDARY           0x26

#define GHOST_ID_RX_RSSI 0x0001            // Rx-side RSSI
#define GHOST_ID_RX_LQ 0x0002              // Rx-side link quality
#define GHOST_ID_RX_SNR 0x0003             // Rx-side signal to noise
#define GHOST_ID_FRAME_RATE 0x0004         // Tx-side frame rate
#define GHOST_ID_TX_POWER 0x0005           // Tx-side power output
#define GHOST_ID_RF_MODE 0x0006            // Tx-side frame rate
#define GHOST_ID_TOTAL_LATENCY 0x0007      // Tx-side total latency
#define GHOST_ID_VTX_FREQ 0x0008           // Vtx Frequency (in MHz)
#define GHOST_ID_VTX_POWER 0x0009          // Vtx Power (in mW)
#define GHOST_ID_VTX_CHAN 0x000A           // Vtx Channel
#define GHOST_ID_VTX_BAND 0x000B           // Vtx Band

#define GHOST_ID_PACK_VOLTS 0x000C         // Battery Pack Voltage
#define GHOST_ID_PACK_AMPS 0x000D          // Battery Pack Current
#define GHOST_ID_PACK_MAH 0x000E           // Battery Pack mAh consumed

#define GHOST_ID_GPS_LAT 0x000F            // GPS Latitude
#define GHOST_ID_GPS_LONG 0x0010           // GPS Longitude
#define GHOST_ID_GPS_ALT 0x0011            // GPS Altitude
#define GHOST_ID_GPS_HDG 0x0012            // GPS Heading
#define GHOST_ID_GPS_GSPD 0x0013           // GPS Ground Speed
#define GHOST_ID_GPS_SATS 0x0014            // GPS Satellite Count

static const char*const rfModeValues[] = {
  "Auto",
  "Norm",
  "Race",
  "Pure",
  "Long",
  "Unused",
  "Race250",
  "Race500",
  "Solid125",
  "Solid250"
};

static const char*const vtxBandNames[] = {
  "- - -" ,
  "IRC",
  "Race",
  "BandE",
  "BandB",
  "BandA"
};
