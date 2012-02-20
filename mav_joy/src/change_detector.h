/*
 * change_handler.h
 *
 *  Created on: Feb 17, 2012
 *      Author: acmarkus
 */

#ifndef CHANGE_HANDLER_H_
#define CHANGE_HANDLER_H_

#include <ros/ros.h>
#include <boost/function.hpp>
#include <map>

template<class T>
  class ChangeDetector
  {
    typedef typename T::value_type T_val;
    typedef boost::function<void(T_val)> CallbackType;
    typedef std::multimap<int, CallbackType> CallbackMap;
    typedef typename CallbackMap::iterator CallbackMapIt;

  private:
    T last_state_;
    CallbackMap callbacks_;

  public:
    template<class T_obj>
      void registerCallback(int channel, void(T_obj::*cb_func)(T_val), T_obj* p_obj)
      {
        callbacks_.insert(std::pair<int, CallbackType>(channel, boost::bind(cb_func, p_obj, _1)));
      }

    void registerCallback(int channel, void(*cb_func)(T_val))
    {
      callbacks_.insert(std::pair<int, CallbackType>(channel, cb_func));
    }

    void init(const T & state)
    {
      last_state_ = state;
    }
    void update(const T & state)
    {
      if (state.size() != last_state_.size())
      {
        last_state_ = state;
        ROS_ERROR("size of container changed, no change detection in this step!");
        return;
      }

      typename T::const_iterator it_new = state.begin();
      typename T::iterator it_old = last_state_.begin();
      for (size_t i = 0; i < state.size(); i++)
      {
//        std::cout << "old: " << (*it_old) << "new:" << (*it_new) << std::endl;
        if ((*it_new) != (*it_old))
        {
          std::pair<CallbackMapIt, CallbackMapIt> ret;
          ret = callbacks_.equal_range(i);
          CallbackMapIt ret_it;
          for (ret_it = ret.first; ret_it != ret.second; ++ret_it)
          {
            (*ret_it).second(*it_new);
          }
        }
        ++it_new;
        ++it_old;
      }
      last_state_ = state;
    }
  };

#endif /* CHANGE_HANDLER_H_ */
