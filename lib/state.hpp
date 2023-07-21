#pragma once

#include <memory>
#include <optional>

enum class ObjectType {
  ARROW_LEFT,
  ARROW_RIGHT,
  CONE,
};

using tObjectSpec = std::tuple<ObjectType, float, float>;
class State {
  private:
  struct dState {
    virtual ~dState() {}
    virtual std::optional<tObjectSpec> internal_object_in_view() = 0;
  };

  template <typename T>
  struct cState : public dState {
    cState(T&& t) : m_value(std::move(t)) {}
    auto internal_object_in_view() -> std::optional<tObjectSpec> override {
      return state_object_in_view(m_value);
    }
    T m_value;
  };

  std::unique_ptr<dState> m_value;
  public:
  template <typename T>
  State(T t) : m_value{new cState<T>(std::move(t))} {}
  friend std::optional<tObjectSpec> object_in_view(State& state) {
    return state.m_value->internal_object_in_view();
  }
};
