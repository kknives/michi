#pragma once

#include <memory>

class Goal {
  private:
  struct dGoal {
    virtual ~dGoal() {}
  };

  template <typename T>
  struct cGoal : public dGoal {
    cGoal(T&& t) : m_value(std::move(t)) {}
    T m_value;
  };

  std::unique_ptr<dGoal> m_value;
  public:
  template <typename T>
  Goal(T t) : m_value{new cGoal<T>(std::move(t))} {}
};
