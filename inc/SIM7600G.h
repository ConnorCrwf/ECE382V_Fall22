/**
 * @file      SIM7600G.h
 * @brief     Provide functions to send text messages via SMS using SIM7600G
 * @details   Configure SIM7600G Cellular Module
 * @author    Pranav Rama, Daniel Valvano, and Jonathan Valvano
 * @copyright Copyright 2022 by Jonathan W. Valvano, valvano@mail.utexas.edu,
 * @warning   AS-IS
 * @note      For more information see  http://users.ece.utexas.edu/~valvano/
 * @date      September 14, 2022

*/
#include <stdint.h>

/**
 * \brief Define for enabling SIM7600G debug output to UART0
 * @note  Remove this line to save time/space
 */
#define SIM7600G_DEBUG 1
/**
 * \brief Save SMS on SIM (small size), SIM7600G_SetSMSStorage
 */
#define SMS_SIM 1
/**
 * \brief Save SMS on SIM7600G flash (large size), SIM7600G_SetSMSStorage
 */
#define SMS_SIM7600G 0

/**
 * @brief reset SIM7600G
 *
 * @param none
 * @return none
 */
void SIM7600G_Reset(void);
void SIM7600G_PowerDown(void);
/**
 * @brief Initialize SIM7600G at 115200 baud
 *  SMS_SIM = 1 for storage on the SIM.<br>
 *  SMS_SIM7600G= 0 for storage on the SIM7600G chip
 * @return none
 */
void SIM7600G_Init(int place);
void SIM7600G_Restart(int place);
/**
 * @brief Send an SMS Message from a buffer provided
 *
 * @param phone   The SMS address buffer
 * @param message The SMS message buffer, null terminated
 * @return 1: success, 0: failure
 */
int SIM7600G_SendSMS(uint8_t phone[],uint8_t message[]);

/**
 * @brief Set the preferred SMS storage
 *  SMS_SIM = 1 for storage on the SIM (small size).<br>
 *  SMS_SIM7600G= 0 for storage on the SIM7600G chip (large size)
 * @param place  1 for SIM, 0 for SIM7600G
 */
void SIM7600G_SetSMSStorage(int place);


/**
 * @brief configure RI pin
 * n=0 off<br>
 * n=1 On(TCPIP,FTP and URC control RI PIN)<br>
 * n=2 On(only TCPIP control RI PIN)
 * @param n  0 1 or 2
 */
void SIM7600G_SetSMSInterrupt(int n);


/**
 * @brief Get the number of SMS messages
 *
 * @param none
 * @return number of messages, -1: failure
 */
int32_t SIM7600G_GetNumSMS(void);

/**
 * @brief Read an SMS message into a provided buffer
 *
 * @param message_index The SMS message index to retrieve, 0 to 255
 * @param smsbuff       Pointer to empty buffer into which message will be filled
 * @param maxlen        The maximum read length (size of smsbuff)
 * @param readlen       The length, number of characters read
 * @return 1: success, 0: failure
 */
int SIM7600G_ReadSMS(uint32_t message_index, char *smsbuff,
                            uint32_t maxlen, uint32_t *readlen);

/**
 * @brief Delete an SMS Message
 *
 * @param message_index The message to delete, 0 to 255
 * @return true: success, false: failure
 */
int SIM7600G_DeleteSMS(uint32_t message_index);

/**
 * @brief Retrieve the sender phone number of the specified SMS message and
 *    copy it as a string to the sender buffer.
 *    Up to senderlen characters will be copied
 *    and a null terminator will be added if less than senderlen
 *    characters are copied to the result.
 *
 * @param message_index The SMS message index to retrieve the sender phone number
 * @param sender        Pointer to an empty buffer into which to fill with the sender phone number
 * @param senderlen     The maximum length to read (size of empty buffer)
 * @return 1: a result was successfully retrieved, 0: failure
 */
int SIM7600G_GetSMSSender(uint32_t message_index, char *sender,
                                 int senderlen);


/** @}*/
