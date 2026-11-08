#pragma once
#include <boost/asio.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>
#include <functional>

using ControlHandler=std::function<nlohmann::json(const std::string &,const nlohmann::json &)>;
// A dedicated WebSocket lane; never shares response workers with map downloads.
void serveControlSocket(boost::asio::ip::tcp::socket socket,
  boost::beast::http::request<boost::beast::http::string_body> request,
  ControlHandler handler,std::function<bool()> alive);
