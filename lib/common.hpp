#pragma once
#include <asio.hpp>
#include <asio/experimental/as_tuple.hpp>
#include <asio/experimental/awaitable_operators.hpp>

using namespace asio::experimental::awaitable_operators;
constexpr auto use_nothrow_awaitable =
  asio::experimental::as_tuple(asio::use_awaitable);
