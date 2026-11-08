#include "control_socket.hpp"
#include <boost/beast/websocket.hpp>
#include <boost/beast/core.hpp>
#include <chrono>

namespace asio=boost::asio;
namespace beast=boost::beast;
namespace websocket=beast::websocket;
using Json=nlohmann::json;

void serveControlSocket(asio::ip::tcp::socket socket,beast::http::request<beast::http::string_body> request,
    ControlHandler handler,std::function<bool()> alive){
  std::string owned;
  try{
    const std::string origin(request[beast::http::field::origin]),host(request[beast::http::field::host]);
    const auto scheme=origin.find("://");
    if(!origin.empty()&&(scheme==std::string::npos||origin.substr(scheme+3)!=host))return;
    // Give the stream its own executor; all async operations run on this lane.
    asio::io_context context;asio::ip::tcp::socket connection(context);
    socket.non_blocking(false);const auto protocol=socket.local_endpoint().protocol();connection.assign(protocol,socket.release());
    websocket::stream<asio::ip::tcp::socket> stream(std::move(connection));
    stream.read_message_max(4096);stream.text(true);
    stream.next_layer().set_option(asio::ip::tcp::no_delay(true));
    beast::flat_buffer buffer;std::string output;
    std::function<void()> read;
    read=[&]{stream.async_read(buffer,[&](beast::error_code error,std::size_t){
      if(error){context.stop();return;}
      Json reply;try{
        const auto body=Json::parse(beast::buffers_to_string(buffer.data()));
        reply["id"]=body.at("id");const auto path=body.at("path").get<std::string>();
        if(path!="/api/control/acquire"&&path!="/api/control/velocity"&&path!="/api/control/release")throw std::invalid_argument("Unsupported control stream operation");
        const auto command=body.at("body");
        if(path!="/api/control/acquire"&&(owned.empty()||command.value("token","")!=owned))throw std::invalid_argument("Control stream does not own this lease");
        auto result=handler(path,command);
        if(path=="/api/control/acquire")owned=result.at("token").get<std::string>();
        if(path=="/api/control/release")owned.clear();
        reply["ok"]=true;reply["result"]=std::move(result);
      }catch(const std::exception &e){reply["ok"]=false;reply["error"]=e.what();}
      buffer.consume(buffer.size());output=reply.dump();
      stream.async_write(asio::buffer(output),[&](beast::error_code error,std::size_t){if(error)context.stop();else read();});
    });};
    stream.async_accept(request,[&](beast::error_code error){if(error)context.stop();else read();});
    while(alive()&&!context.stopped())context.run_for(std::chrono::milliseconds(20));
    beast::error_code ignored;stream.next_layer().close(ignored);context.stop();
  }catch(const std::exception &){ /* Cleanup below also handles incomplete frames and disconnects. */ }
  if(!owned.empty())try{handler("/api/control/release",{{"token",owned}});}catch(const std::exception &){}
}
