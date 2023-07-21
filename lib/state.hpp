#pragma once

#include <memory>

class State {
  private:
  struct dState {
    virtual ~dState() {}
  };

  template <typename T>
  struct cState : public dState {
    cState(T&& t) : m_value(std::move(t)) {}
    T m_value;
  };

  std::unique_ptr<dState> m_value;
  public:
  template <typename T>
  State(T t) : m_value{new cState<T>(std::move(t))} {}
};
