#include "control_socket.hpp"
#include <boost/beast/websocket.hpp>
#include <boost/beast/core.hpp>
#include <gtest/gtest.h>
#include <atomic>
#include <thread>

namespace asio=boost::asio;namespace websocket=boost::beast::websocket;
using tcp=asio::ip::tcp;using Json=nlohmann::json;
TEST(ControlSocket, PersistentCommandsAndDisconnectReleaseOnlyTheirOwnLease){
  asio::io_context context;tcp::acceptor listener(context,{tcp::v4(),0});
  std::atomic<bool> alive{true};std::atomic<int> velocities{0},releases{0};
  std::thread server([&]{tcp::socket socket(context);listener.accept(socket);
    boost::beast::flat_buffer buffer;boost::beast::http::request<boost::beast::http::string_body> request;
    boost::beast::http::read(socket,buffer,request);
    serveControlSocket(std::move(socket),std::move(request),[&](const std::string &path,const Json &body)->Json{
      if(path=="/api/control/acquire")return {{"token","owned"}};
      if(body.at("token")!="owned")throw std::invalid_argument("wrong lease");
      if(path=="/api/control/velocity")++velocities;
      if(path=="/api/control/release")++releases;
      return {{"success",true}};
    },[&]{return alive.load();});
  });
  websocket::stream<tcp::socket> client(context);client.next_layer().connect({asio::ip::make_address("127.0.0.1"),listener.local_endpoint().port()});
  client.handshake("127.0.0.1","/api/control/stream");
  auto call=[&](const std::string &path,const Json &body){client.write(asio::buffer(Json({{"id",1},{"path",path},{"body",body}}).dump()));boost::beast::flat_buffer b;client.read(b);return Json::parse(boost::beast::buffers_to_string(b.data()));};
  EXPECT_TRUE(call("/api/control/acquire",Json::object())["ok"].get<bool>());
  for(int i=0;i<30;++i)EXPECT_TRUE(call("/api/control/velocity",{{"token","owned"},{"x",.2},{"y",0},{"yaw",0}})["ok"].get<bool>());
  EXPECT_FALSE(call("/api/control/release",{{"token","someone_else"}})["ok"].get<bool>());
  EXPECT_EQ(releases.load(),0);client.close(websocket::close_code::normal);server.join();
  EXPECT_EQ(velocities.load(),30);EXPECT_EQ(releases.load(),1);
}

TEST(ControlSocket, ShutdownInterruptsAnIdleOrPartiallyReadStream){
  asio::io_context context;tcp::acceptor listener(context,{tcp::v4(),0});std::atomic<bool> alive{true};
  std::thread server([&]{tcp::socket socket(context);listener.accept(socket);
    boost::beast::flat_buffer buffer;boost::beast::http::request<boost::beast::http::string_body> request;
    boost::beast::http::read(socket,buffer,request);
    serveControlSocket(std::move(socket),std::move(request),[](const std::string &,const Json &){return Json::object();},[&]{return alive.load();});
  });
  websocket::stream<tcp::socket> client(context);client.next_layer().connect({asio::ip::make_address("127.0.0.1"),listener.local_endpoint().port()});client.handshake("127.0.0.1","/api/control/stream");
  const auto begin=std::chrono::steady_clock::now();alive=false;server.join();
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count(),1.);
}

TEST(ControlSocket, RejectsCrossOriginUpgradeBeforeAnyCommand){
  asio::io_context context;tcp::acceptor listener(context,{tcp::v4(),0});std::atomic<int> commands{0};
  std::thread server([&]{tcp::socket socket(context);listener.accept(socket);
    boost::beast::flat_buffer buffer;boost::beast::http::request<boost::beast::http::string_body> request;
    boost::beast::http::read(socket,buffer,request);
    serveControlSocket(std::move(socket),std::move(request),[&](const std::string &,const Json &){++commands;return Json::object();},[]{return true;});
  });
  websocket::stream<tcp::socket> client(context);client.next_layer().connect({asio::ip::make_address("127.0.0.1"),listener.local_endpoint().port()});
  client.set_option(websocket::stream_base::decorator([](websocket::request_type &request){request.set(boost::beast::http::field::origin,"https://untrusted.example");}));
  EXPECT_THROW(client.handshake("127.0.0.1","/api/control/stream"),boost::system::system_error);server.join();EXPECT_EQ(commands.load(),0);
}
