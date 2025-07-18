/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network:Service
 * Copyright (c) 2004-2018 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    HTTP_Server_CGI.c
 * Purpose: HTTP Server CGI Module
 * Rev.:    V6.0.0
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE
#include "Principal_Master.h"

//#include "Board_LED.h"                  // ::Board Support:LED
#include "LEDs.h"

#if      defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma  clang diagnostic push
#pragma  clang diagnostic ignored "-Wformat-nonliteral"
#endif

// http_server.c

extern uint16_t AD_in (uint32_t ch);
extern uint8_t  get_button (void);

extern bool LEDrun;
extern bool full;

extern MEDIDAS_t meas_global;
extern char web_buffer[BUFFER_SIZE][STRING_SIZE];
uint8_t aux_index = 0;
extern uint8_t temp_index;
extern uint8_t temp_read_index;
extern bool web;

extern char lcd_text[2][20+1];
extern osThreadId_t TID_Display;
extern osThreadId_t TID_PRINC_MASTER;

// Local variables.
static uint8_t P2;
static uint8_t ip_addr[NET_ADDR_IP6_LEN];
static char    ip_string[40];

extern Estados_Principal_t estadoMaster;

// My structure of CGI status variable.
typedef struct {
  uint8_t idx;
  uint8_t unused[3];
} MY_BUF;
#define MYBUF(p)        ((MY_BUF *)p)

// Process query string received by GET request.
void netCGI_ProcessQuery (const char *qstr) {
  netIF_Option opt = netIF_OptionMAC_Address;
  int16_t      typ = 0;
  char var[40];

  do {
    // Loop through all the parameters
    qstr = netCGI_GetEnvVar (qstr, var, sizeof (var));
    // Check return string, 'qstr' now points to the next parameter

    switch (var[0]) {
      case 'i': // Local IP address
        if (var[1] == '4') { opt = netIF_OptionIP4_Address;       }
        else               { opt = netIF_OptionIP6_StaticAddress; }
        break;

      case 'm': // Local network mask
        if (var[1] == '4') { opt = netIF_OptionIP4_SubnetMask; }
        break;

      case 'g': // Default gateway IP address
        if (var[1] == '4') { opt = netIF_OptionIP6_DefaultGateway; }
        else               { opt = netIF_OptionIP6_DefaultGateway; }
        break;

      case 'p': // Primary DNS server IP address
        if (var[1] == '4') { opt = netIF_OptionIP4_PrimaryDNS; }
        else               { opt = netIF_OptionIP6_PrimaryDNS; }
        break;

      case 's': // Secondary DNS server IP address
        if (var[1] == '4') { opt = netIF_OptionIP4_SecondaryDNS; }
        else               { opt = netIF_OptionIP6_SecondaryDNS; }
        break;
      
      default: var[0] = '\0'; break;
    }

    switch (var[1]) {
      case '4': typ = NET_ADDR_IP4; break;
      case '6': typ = NET_ADDR_IP6; break;

      default: var[0] = '\0'; break;
    }

    if ((var[0] != '\0') && (var[2] == '=')) {
      netIP_aton (&var[3], typ, ip_addr);
      // Set required option
      netIF_SetOption (NET_IF_CLASS_ETH, opt, ip_addr, sizeof(ip_addr));
    }
  } while (qstr);
}

// Process data received by POST request.
// Type code: - 0 = www-url-encoded form data.
//            - 1 = filename for file upload (null-terminated string).
//            - 2 = file upload raw data.
//            - 3 = end of file upload (file close requested).
//            - 4 = any XML encoded POST data (single or last stream).
//            - 5 = the same as 4, but with more XML data to follow.
void netCGI_ProcessData (uint8_t code, const char *data, uint32_t len) {
  char var[40],passw[12];

  if (code != 0) {
    // Ignore all other codes
    return;
  }

  P2 = 0;
  LEDrun = true;
  if (len == 0) {
    return;
  }
  passw[0] = 1;
  do {
    // Parse all parameters
    data = netCGI_GetEnvVar (data, var, sizeof (var));
    if (var[0] != 0) {
      // First character is non-null, string exists
      if (strcmp (var, "led0=on") == 0) {		// Compara la cadena var con led0=on, y si son iguales devuelve un 0
        P2 |= 0x01;													// Hace una OR de P2 con 0x01, ya que esta linea es lo mismo que poner P2 = P2 0R 0x01
      }
      else if (strcmp (var, "led1=on") == 0) {
        P2 |= 0x02;
      }
      else if (strcmp (var, "led2=on") == 0) {
        P2 |= 0x04;
      }
      else if (strcmp (var, "led3=on") == 0) {
        P2 |= 0x08;
      }
      else if (strcmp (var, "led4=on") == 0) {
        P2 |= 0x10;
      }
      else if (strcmp (var, "led5=on") == 0) {
        P2 |= 0x20;
      }
      else if (strcmp (var, "ctrl=Browser") == 0) {		// Cuando seleccionamos el modo Browser, se dejan de encender los LEDs solos.
        LEDrun = false;
      }
      else if ((strncmp (var, "pw0=", 4) == 0) ||
               (strncmp (var, "pw2=", 4) == 0)) {
        // Change password, retyped password
        if (netHTTPs_LoginActive()) {
          if (passw[0] == 1) {
            strcpy (passw, var+4);
          }
          else if (strcmp (passw, var+4) == 0) {
            // Both strings are equal, change the password
            netHTTPs_SetPassword (passw);
          }
        }
      }
      else if (strncmp (var, "lcd1=", 5) == 0) {	// Comparamos que los primeros 5 caracteres de var sean iguales que lcd1=
        // LCD Module line 1 text
        strcpy (lcd_text[0], var+5);							// Copia el contenido que hay en var+5, en lcd_text[0]. Le sumamos 5 a var, porque sino se nos pintar�a siempre lcd1= antes de lo que escribamos
        osThreadFlagsSet (TID_Display, 0x01);			// Envio flag al LCD de que se quiere escribir algo
      }
      else if (strncmp (var, "lcd2=", 5) == 0) {	// Comparamos que los primeros 5 caracteres de var sean iguales que lcd2=
        // LCD Module line 2 text
        strcpy (lcd_text[1], var+5);							// Copia el contenido que hay en var+5, en lcd_text[1]
        osThreadFlagsSet (TID_Display, 0x01);			// Envio flag al LCD de que se quiere escribir algo
      }
      else if (strncmp (var, "lowMode1=", 9) == 0){
        LEDrun = false; 
        web = false;
      }
      else if(strncmp (var, "exitLow=", 8) == 0){
        if(estadoMaster != PRESENCIAL)
          web = true;
        osThreadFlagsSet(TID_PRINC_MASTER, 0x40);
      }
    }
  } while (data);
}

// Generate dynamic web data from a script line.
uint32_t netCGI_Script (const char *env, char *buf, uint32_t buflen, uint32_t *pcgi) {
  int32_t socket;
  netTCP_State state;
  NET_ADDR r_client;
  const char *lang;
  uint32_t len = 0U;
  uint8_t id;
  static float adv1, adv2;
  static float lux;
  static float wl;
  static double hum;
  static int aq;
  netIF_Option opt = netIF_OptionMAC_Address;
  int16_t      typ = 0;

  switch (env[0]) {
    // Analyze a 'c' script line starting position 2
    case 'a' :
      // Network parameters from 'network.cgi'
      switch (env[3]) {
        case '4': typ = NET_ADDR_IP4; break;
        case '6': typ = NET_ADDR_IP6; break;

        default: return (0);
      }
      
      switch (env[2]) {
        case 'l':
          // Link-local address
          if (env[3] == '4') { return (0);                             }
          else               { opt = netIF_OptionIP6_LinkLocalAddress; }
          break;

        case 'i':
          // Write local IP address (IPv4 or IPv6)
          if (env[3] == '4') { opt = netIF_OptionIP4_Address;       }
          else               { opt = netIF_OptionIP6_StaticAddress; }
          break;

        case 'm':
          // Write local network mask
          if (env[3] == '4') { opt = netIF_OptionIP4_SubnetMask; }
          else               { return (0);                       }
          break;

        case 'g':
          // Write default gateway IP address
          if (env[3] == '4') { opt = netIF_OptionIP4_DefaultGateway; }
          else               { opt = netIF_OptionIP6_DefaultGateway; }
          break;

        case 'p':
          // Write primary DNS server IP address
          if (env[3] == '4') { opt = netIF_OptionIP4_PrimaryDNS; }
          else               { opt = netIF_OptionIP6_PrimaryDNS; }
          break;

        case 's':
          // Write secondary DNS server IP address
          if (env[3] == '4') { opt = netIF_OptionIP4_SecondaryDNS; }
          else               { opt = netIF_OptionIP6_SecondaryDNS; }
          break;
      }

      netIF_GetOption (NET_IF_CLASS_ETH, opt, ip_addr, sizeof(ip_addr));
      netIP_ntoa (typ, ip_addr, ip_string, sizeof(ip_string));
      len = (uint32_t)sprintf (buf, &env[5], ip_string);
      break;

    case 'b':
      // AD Input from 'ad.cgi'
      switch (env[2]) {
        /*Flag para salir del bajo consumo y enviar SET*/
        case '1':
					adv1 = meas_global.temperatura;
          len = (uint32_t)sprintf (buf, &env[4], adv1);  // -5 -2, donde el -2 corresponde al error de no linealidad
          break;
				case '2':
//          osThreadFlagsSet(TID_Lux, BRIGHT_MEASURE);
					hum = meas_global.humidity;
					len = (uint32_t)sprintf (buf, &env[1], hum);
          break;
        case '3':
          lux = meas_global.lum;
					len = (uint32_t)sprintf (buf, &env[1], lux);
          break;
				case '4':
//          osThreadFlagsSet(TID_Lux, BRIGHT_MEASURE);
					aq = meas_global.air_quality;
					len = (uint32_t)sprintf (buf, &env[1], aq);
          break;
        case '5':
//          osThreadFlagsSet(TID_Lux, BRIGHT_MEASURE);
					wl = meas_global.quantity;
					len = (uint32_t)sprintf (buf, &env[1], wl);
          break;
      }
      break;

    case 'c':
      // TCP status from 'tcp.cgi'
      while ((uint32_t)(len + 150) < buflen) {
        socket = ++MYBUF(pcgi)->idx;
        state  = netTCP_GetState (socket);

        if (state == netTCP_StateINVALID) {
          /* Invalid socket, we are done */
          return ((uint32_t)len);
        }

        // 'sprintf' format string is defined here
        len += (uint32_t)sprintf (buf+len,   "<tr align=\"center\">");
        if (state <= netTCP_StateCLOSED) {
          len += (uint32_t)sprintf (buf+len, "<td>%d</td><td>%d</td><td>-</td><td>-</td>"
                                             "<td>-</td><td>-</td></tr>\r\n",
                                             socket,
                                             netTCP_StateCLOSED);
        }
        else if (state == netTCP_StateLISTEN) {
          len += (uint32_t)sprintf (buf+len, "<td>%d</td><td>%d</td><td>%d</td><td>-</td>"
                                             "<td>-</td><td>-</td></tr>\r\n",
                                             socket,
                                             netTCP_StateLISTEN,
                                             netTCP_GetLocalPort(socket));
        }
        else {
          netTCP_GetPeer (socket, &r_client, sizeof(r_client));

          netIP_ntoa (r_client.addr_type, r_client.addr, ip_string, sizeof (ip_string));
          
          len += (uint32_t)sprintf (buf+len, "<td>%d</td><td>%d</td><td>%d</td>"
                                             "<td>%d</td><td>%s</td><td>%d</td></tr>\r\n",
                                             socket, netTCP_StateLISTEN, netTCP_GetLocalPort(socket),
                                             netTCP_GetTimer(socket), ip_string, r_client.port);
        }
      }
      /* More sockets to go, set a repeat flag */
      len |= (1u << 31);
      break;

    case 'd':
      // System password from 'system.cgi'
      switch (env[2]) {
        case '1':
          len = (uint32_t)sprintf (buf, &env[4], netHTTPs_LoginActive() ? "Enabled" : "Disabled");
          break;
        case '2':
          len = (uint32_t)sprintf (buf, &env[4], netHTTPs_GetPassword());
          break;
      }
      break;

    case 'f':
      // LCD Module control from 'lcd.cgi'
      switch (env[2]) {
        case '1':
          len = (uint32_t)sprintf (buf, &env[4], lcd_text[0]);
          break;
        case '2':
          len = (uint32_t)sprintf (buf, &env[4], lcd_text[1]);
          break;
      }
      break;

    case 'e':
      // Consumption control from 'consumption.cgi'
      switch (env[2]) {
        case '1':
					adv2 = meas_global.consumo;
          len = (uint32_t)sprintf (buf, &env[4], adv2);  // mA
          break;
      }
      break;

    case 'k':
      switch (env[2]) {
        case '1':
          if(full == false){
            if(temp_index != temp_read_index){
              for(uint8_t contador = temp_read_index; contador != temp_index; contador++){
                if(contador > 9){
                  contador = 0;
                }
                leer_una_medida_y_mostrar();
              }
              for(uint8_t i = 0; i < 10; i++){
                len += (uint32_t)sprintf (buf+len, "<tr><td>%s</td></tr>\r\n", web_buffer[i]);
              }
            }else{
              for(uint8_t i = 0; i < 10; i++){
                len += (uint32_t)sprintf (buf+len, "<tr><td>%s</td></tr>\r\n", web_buffer[i]);
              }
            }
          }else{
            for(uint8_t contador = temp_index + 1; contador != temp_index; contador++){
              if(contador > 9){
                  contador = 0;
                }
                leer_una_medida_y_mostrar();
              }
              for(uint8_t i = 0; i < 10; i++){
                len += (uint32_t)sprintf (buf+len, "<tr><td>%s</td></tr>\r\n", web_buffer[i]);
              }
          }
          break;
      }
      break;

    case 'u':
      // RTD Input from 'rtm.cgx'
      if (estadoMaster == WEB){
        adv1 = meas_global.temperatura;
        len = (uint32_t)sprintf (buf, &env[1], adv1);
      }else{
        adv1 = 0.0;
        len = (uint32_t)sprintf (buf, &env[1], adv1);
      }
      break;

    case 'v':
      // Luminosity Input from 'rtm.cgx'
      if (estadoMaster == WEB){
        hum = meas_global.humidity;
        len = (uint32_t)sprintf (buf, &env[1], hum);
      }else{
        hum = 0.0;
        len = (uint32_t)sprintf (buf, &env[1], hum);
      }
      break;

    case 'w':
      // AD Input from 'rtm.cgx'
      if (estadoMaster == WEB){
        lux = meas_global.lum;
        len = (uint32_t)sprintf (buf, &env[1], lux);
      }else{
        lux = 0.0;
        len = (uint32_t)sprintf (buf, &env[1], lux);
      }
      break;

    case 'x':
      // AD Input from 'rtm.cgx'
      if (estadoMaster == WEB){
        aq = meas_global.air_quality;
        len = (uint32_t)sprintf (buf, &env[1], aq);
      }else{
        aq = 0;
        len = (uint32_t)sprintf (buf, &env[1], aq);
      }
      break;
		
		case 'y':
      // AD Input from 'rtm.cgx'
      if (estadoMaster == WEB){
        wl = meas_global.quantity;
        len = (uint32_t)sprintf (buf, &env[1], wl);
      }else{
        wl = 0.0;
        len = (uint32_t)sprintf (buf, &env[1], wl);
      }
      break;
    
    case 'z':
      // AD Input from 'consumption.cgx'
      if (estadoMaster == STANBY){
		  adv2 = 150.6f;
      }else{
      adv2 = meas_global.consumo;
      }
      len = (uint32_t)sprintf (buf, &env[1], adv2);
      break;
		
  }
  return (len);
}

#if      defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma  clang diagnostic pop
#endif
