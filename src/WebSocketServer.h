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

#pragma once

#include <mutex>
#include <set>

#include "WebSocketConnection.h"
#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

class WebSocketServer
{
public:

	using ConnectionHandle = websocketpp::connection_hdl;
	using ConnectionSet = std::set<websocketpp::connection_hdl,std::owner_less<websocketpp::connection_hdl>>;
	using Server = websocketpp::server<websocketpp::config::asio>;
	using ConnectionRef = Server::connection_ptr;
	using MessageRef = Server::message_ptr;

	WebSocketServer();
	WebSocketServer( const std::shared_ptr<asio::io_service>& service );
	~WebSocketServer();
	
	void			cancel();
	void			listen( uint16_t port = 80 );
	void			ping( ConnectionHandle client, const std::string& msg = "" );
	void			pingAll( const std::string& msg = "");
	void			poll();
	void			run();
	void			send( ConnectionHandle client, const std::string& msg );
	void			send( ConnectionHandle client, void const * msg, size_t len );
	void            broadcast( const std::string& msg );
	void			broadcast( void const * msg, size_t len );
	void			close();

	std::shared_ptr<asio::io_service> mIoService;
	Server&			getServer();
	const Server&	getServer() const;

	inline void		connectOpenConnectionHandler(const std::function<void(ConnectionHandle)>& handler) { mOnOpenConnection = handler; }
	inline void		connectCloseConnectionHandler(const std::function<void(ConnectionHandle)>& handler) { mOnCloseConnection = handler; }
	inline void		connectErrorHandler(const std::function<void(ConnectionHandle,const std::string&)>& handler) { mOnConnectionError = handler; }
	inline void		connectHttpHandler(const std::function<void(ConnectionHandle)>& handler) { mOnHttp = handler; }
	inline void		connectMessageHandler(const std::function<void(ConnectionHandle, const std::string&)>& handler) { mOnMessage = handler; }
	inline void		connectPingReceivedHandler(const std::function<void(ConnectionHandle, const std::string&)>& handler) { mOnPingReceived = handler; }
	inline void		connectPongReceivedHandler(const std::function<void(ConnectionHandle, const std::string&)>& handler) { mOnPongReceived = handler; }
	inline void		connectWriteHandler(const std::function<void(ConnectionHandle)>& handler) { mOnWrite = handler; }

	inline void		disconnectOpenConnectionHandler() { mOnOpenConnection = nullptr; }
	inline void		disconnectCloseConnectionHandler() { mOnCloseConnection = nullptr; }
	inline void		disconnectErrorHandler() { mOnConnectionError = nullptr; }
	inline void		disconnectHttpHandler() { mOnHttp = nullptr; }
	inline void		disconnectMessageHandler() { mOnMessage = nullptr; }
	inline void		disconnectPingReceivedHandler() { mOnPingReceived = nullptr; }
	inline void		disconnectPongReceivedHandler() { mOnPongReceived = nullptr; }
	inline void		disconnectWriteHandler() { mOnWrite = nullptr; }

protected:

	void	onCloseConnection(websocketpp::connection_hdl);
	void	onOpenConnection(websocketpp::connection_hdl);
	void	onConnectionError(websocketpp::connection_hdl);
	void	onHttp(websocketpp::connection_hdl);
	void	onMessage(websocketpp::connection_hdl, MessageRef);
	bool	onPingReceived(websocketpp::connection_hdl, std::string);
	void	onPongReceived(websocketpp::connection_hdl, std::string);

	Server														mServer;
	ConnectionSet												mConnections;

	std::function<void(ConnectionHandle)>						mOnOpenConnection;
	std::function<void(ConnectionHandle)>						mOnCloseConnection;
	std::function<void(ConnectionHandle, const std::string&)>	mOnConnectionError;
	std::function<void(ConnectionHandle)>						mOnHttp;
	std::function<void(ConnectionHandle, const std::string&)>	mOnMessage;
	std::function<void(ConnectionHandle, const std::string&)>	mOnPingReceived;
	std::function<void(ConnectionHandle, const std::string&)>	mOnPongReceived;
	std::function<void(ConnectionHandle)>						mOnWrite;

};
