/*
 * Copyright (c) 2015, Wieden+Kennedy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of the Ban the Rewind nor the names of its
 * contributors may be used to endorse or promote products
 * derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "WebSocketServer.h"

#include "cinder/Log.h"
#include "cinder/Utilities.h"
#include "cinder/Log.h"

using namespace ci;
using namespace std;

static uint32_t sNextClient = 0;
uint32_t getNextClientId() {
	return sNextClient++;
}

WebSocketServer::WebSocketServer()
{
	mServer.set_access_channels( websocketpp::log::alevel::all );
	mServer.clear_access_channels( websocketpp::log::alevel::frame_payload );
	
	mServer.init_asio();

	mServer.set_fail_handler( bind( &WebSocketServer::onConnectionError, this, std::placeholders::_1 ) );
	mServer.set_http_handler( bind( &WebSocketServer::onHttp, this, std::placeholders::_1 ) );
	mServer.set_message_handler( bind( &WebSocketServer::onMessage,	this, std::placeholders::_1, std::placeholders::_2 ) );
	mServer.set_open_handler( bind( &WebSocketServer::onOpenConnection, this, std::placeholders::_1 ) );
	mServer.set_close_handler( bind(&WebSocketServer::onCloseConnection, this, std::placeholders::_1));
	mServer.set_ping_handler( bind( &WebSocketServer::onPingReceived, this, std::placeholders::_1, std::placeholders::_2 ) );
	mServer.set_pong_handler( bind(&WebSocketServer::onPongReceived, this, std::placeholders::_1, std::placeholders::_2) );
}

WebSocketServer::~WebSocketServer()
{
	if ( !mServer.stopped() ) {
		cancel();
		mServer.stop();
	}
}

void WebSocketServer::cancel()
{
	try {
		mServer.stop_listening();
	} catch ( const std::exception& ex ) {
		CI_LOG_E( ex.what() );
    } catch ( websocketpp::lib::error_code err ) {
		CI_LOG_E( err.message() );
    } catch ( ... ) {
		CI_LOG_E( "An unknown exception occurred." );
    }
}

void WebSocketServer::listen( uint16_t port )
{
	try {
		mServer.listen( port );
		mServer.start_accept();
	}
	catch (const std::exception& ex) {
		CI_LOG_E(ex.what());
	}
	catch (websocketpp::lib::error_code err) {
		CI_LOG_E(err.message());
	}
	catch (...) {
		CI_LOG_E("An unknown exception occurred.");
	}
}

void WebSocketServer::ping(ConnectionHandle client, const std::string & msg)
{
	try {
		mServer.get_con_from_hdl( client )->pong( msg );
	} catch( ... ) {
		if (mOnConnectionError) {
			mOnConnectionError( client, "Ping failed." );
		}
	}
}

void WebSocketServer::pingAll(const std::string & msg)
{
	for ( auto con : mConnections ) {
		ping(con,msg);
	}
}

void WebSocketServer::poll()
{
	mServer.poll();
}

void WebSocketServer::run()
{
	mServer.run();
}

void WebSocketServer::broadcast(const std::string & msg)
{
	for (auto con : mConnections) {
		send(con, msg);
	}
}

void WebSocketServer::broadcast(void const * msg, size_t len)
{
	for ( auto con : mConnections) {
		send(con, msg, len);
	}
}

void WebSocketServer::send( ConnectionHandle client, void const * msg, size_t len )
{
	if (len == 0){
		CI_LOG_E("Message cannot be empty");
	} else {
		websocketpp::lib::error_code err;
		mServer.send(client,
					 msg,
					 len,
					 websocketpp::frame::opcode::BINARY,
					 err);
		if (err) {
			if (mOnConnectionError) {
				mOnConnectionError(client, err.message());
			}
		} else {
			if (mOnWrite) {
				mOnWrite(client);
			}
		}
	}
}

void WebSocketServer::send( ConnectionHandle client, const std::string& msg )
{
	if ( msg.empty() ) {
		CI_LOG_E("Message cannot be empty");
	} else {
		websocketpp::lib::error_code err;
		mServer.send(client, msg, websocketpp::frame::opcode::TEXT, err );
		if ( err ) {
			if (mOnConnectionError) {
				mOnConnectionError(client, err.message());
			}
		}
		else {
			if (mOnWrite) {
				mOnWrite(client);
			}
		}
	}
}

WebSocketServer::Server& WebSocketServer::getServer()
{
	return mServer;
}

const WebSocketServer::Server& WebSocketServer::getServer() const
{
	return mServer;
}

void WebSocketServer::onCloseConnection( websocketpp::connection_hdl handle )
{
	if ( mOnCloseConnection ) {
		mOnCloseConnection(handle);
	}
}

void WebSocketServer::onConnectionError( websocketpp::connection_hdl handle )
{
	if ( mOnConnectionError ) {
		mOnConnectionError( handle, "Transfer failed." );
	}
}

void WebSocketServer::onHttp( websocketpp::connection_hdl handle )
{
	if ( mOnHttp) {
		mOnHttp(handle);
	}
}

void WebSocketServer::onMessage( websocketpp::connection_hdl handle, MessageRef msg )
{
	if ( mOnMessage ) {
		mOnMessage( handle, msg->get_payload() );
	}
}

void WebSocketServer::onOpenConnection( websocketpp::connection_hdl handle )
{
	auto success = mConnections.insert(handle);
	if (!success.second) {
		CI_LOG_W("Already have this connection?");
	}
	else {
		if (mOnOpenConnection != nullptr) {
			mOnOpenConnection(handle);
		}
	}
}

bool WebSocketServer::onPingReceived( websocketpp::connection_hdl handle, std::string msg )
{
	if ( mOnPingReceived ) {
		mOnPingReceived( handle, msg );
	}
	return true;
}

void WebSocketServer::onPongReceived(websocketpp::connection_hdl client, std::string msg)
{
	if (mOnPongReceived) {
		mOnPongReceived(client, msg);
	}
}


