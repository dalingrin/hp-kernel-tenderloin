/* -------------------------------------------------------------------------
 * Copyright (C) 2010 Inside Secure
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ------------------------------------------------------------------------- */

#ifndef CCCLIENT_H
#define CCCLIENT_H


/*******************************************************************************
  Error codes
*******************************************************************************/

typedef enum __ccErrorCode {

   CC_SUCCESS                    =  0,    /* successfull connection */
   CC_ERROR_URI_SYNTAX           =  1,    /* syntax error in URI parameter */
   CC_ERROR_SOCKET_OPERATION     =  2,    /* error creating or configuring the TCP/IP socket */
   CC_ERROR_CONNECTION_FAILURE   =  3,    /* error connecting to the connection center */
   CC_ERROR_MEMORY               =  4,    /* memory allocation failure */
   CC_ERROR_EVENT_CREATION       =  5,    /* error creating or configuring the socket Event */
   CC_ERROR_TX_RX_MESSAGE        =  6,    /* error sending to / receiving from the Connection Center */
   CC_ERROR_PROTOCOL             =  7,    /* error in message received from Connection Center */
   CC_ERROR_CC_VERSION           =  8,    /* Connection Center with incompatible version */
   CC_ERROR_NO_PROVIDER          =  9,    /* Connected to Connection Center, but no Service Provider yet connected */
   CC_ERROR_PROVIDER_BUSY        = 10,    /* Connected to Connection Center, but Service Provider is already used */
} ccErrorCode;

/**
 * Extracts the protocol from the URI.
 *
 * The syntax of the URI is the following:
 *
 *    cc:[//<host>/]<protocol>[?[<query>][;<query>]*]
 *
 * where
 *   host ::= localhost|<host name>|<host address>
 *   query ::= <name=value> | <process=id>
 *
 * @param[in]  pProviderURI  The provider URI.
 *
 * @param[in]  pProtocolBuffer  The buffer receiving the protocol.
 *
 * @param[in]  nProtocolBufferLength  The buffer length in character.
 *
 * @return  true in case of success, false in case of error.
 **/
bool CCClientGetProtocol(
            const tchar* pProviderURI,
            tchar* pProtocolBuffer,
            uint32_t nProtocolBufferLength);

/**
 * Opens a connection as client.
 *
 * @param[in] pProviderURI  The provider URI.
 *
 * @param[in] bWait         Wait for service provider (true) or disconnect if no service provider is ready (false)
 *
 * @param[out] pError  The error, or CC_SUCCESS.
 *
 * @return The connection or null in case of error.
 **/
void* CCClientOpen(
            uint32_t       nInetAddr,
            const tchar* pProviderURI,
            bool bWait,
            uint32_t* pError);

/**
 * Opens a connection as service provider.
 *
 * @param[in] pServiceType  The service type.
 *
 * @param[in] pServiceName  The service name. This value may be null for the default name.
 *
 * @param[out] pError  The error, or CC_SUCCESS.
 *
 * @return The connection or null in case of error.
 **/
void* CCClientOpenAsProvider(
            uint32_t       nInetAddr,
            const tchar* pServiceType,
            const tchar* pServiceName,
            uint32_t* pError);

void CCClientDisconnect(
            void * pConnection);
/**
 * Closes a connection.
 *
 * @param[in]  pConnection  The connection.
 **/
void CCClientClose(
            void* pConnection);

/**
 * Sends some binary data.
 *
 * @param[in] pConnection  The connection.
 *
 * @param[in] pData  A pointer on the data buffer.
 *
 * @param[in] nDataLength  The length in bytes of the data.
 *
 * @return  Non zero value if the data is sent, zero if an error occured.
 **/
uint32_t CCClientSendData(
            void* pConnection,
            const uint8_t* pData,
            uint32_t nDataLength);

/**
 * Sends some binary data.
 *
 * @param[in] pConnection  The connection.
 *
 * @param[in] nData1  The value of the first data buffer.
 *
 * @param[in] pData2  A pointer on the second data buffer.
 *
 * @param[in] nData2Length  The length in bytes of the second data buffer.
 *
 * @return  Non zero value if the data is sent, zero if an error occured.
 **/
uint32_t CCClientSendDataEx(
            void* pConnection,
            const uint8_t nData1,
            const uint8_t* pData2,
            uint32_t nData2Length);

/**
 * Receives some binary data.
 *
 * @param[in]  pConnection  The connection.
 *
 * @param[in]  pBuffer  A pointer on the buffer used for the reception.
 *
 * @param[in]  nBufferLength  The length in bytes of the buffer.
 *
 * @param[out] ppData  A pointer on a variable valued with the address
 *             of the data in the buffer.
 *
 * @return  The length in bytes of the data,
 *          zero if no data is received,
 *          a negative value if an error is detected.
 **/
sint32_t CCClientReceiveData(
            void* pConnection,
            uint8_t* pBuffer,
            uint32_t nBufferLength,
            uint8_t** ppData,
            bool        bWait);

/**
 * Gets the reception event.
 *
 * @param[in]  pConnection  The connection.
 *
 * @return The reception event.
 **/
int CCClientGetReceptionEvent(
            void* pConnection);


#define MSG_ROUTER_HELLO               'A'
#define MSG_PROVIDER_CONNECT           'B'
#define MSG_CLIENT_CONNECT             'C'
#define MSG_CLIENT_CONNECT_WAIT        'D'
#define MSG_CONNECT_OK                 'E'
#define MSG_CONNECT_ERR_NO_PROVIDER    'F'
#define MSG_CONNECT_ERR_PROVIDER_BUSY  'G'
#define MSG_DISCONNECT                 'H'
#define MSG_SPECIFIC_MESSAGE           'K'
#define MSG_ADMIN                      'M'

#define CCCLIENT_SEND_BUFFER_LENGTH  4096
#define CCCLIENT_RECV_BUFFER_LENGTH  4096

#endif


