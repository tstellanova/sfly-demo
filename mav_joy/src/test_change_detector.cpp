/*
 * test_change_detector.cpp
 *
 *  Created on: Feb 17, 2012
 *      Author: acmarkus
 */

#include "change_detector.h"

class Test
{
public:
  void takeoffCallback(int val)
  {
    std::cout << "takeoff: " << val << std::endl;
  }

  void filterInitCallback(int val)
  {
    std::cout << "filterinit: " << val << std::endl;
  }

  void ptamSpaceCallback(int val)
  {
    std::cout << "ptamspace: " << val << std::endl;
  }

  void ptamResetCallback(int val)
  {
    std::cout << "ptamreset: " << val << std::endl;
  }

};

void testfunc(int val){
    std::cout<<"test: "<<val<<std::endl;
}


int main(int argc, char** argv)
{

  ChangeDetector<std::vector<int> > button_handler_;
  Test testclass;

  button_handler_.registerCallback(0, &Test::takeoffCallback,&testclass);
  button_handler_.registerCallback(0, &Test::filterInitCallback,&testclass);
  button_handler_.registerCallback(3, &Test::ptamSpaceCallback,&testclass);
  button_handler_.registerCallback(5, &Test::ptamResetCallback,&testclass);
  button_handler_.registerCallback(2,testfunc);

  std::vector<int> test_vec;
  test_vec.assign(6,0);
  button_handler_.init(test_vec);

  button_handler_.update(test_vec);
  std::cout<<"---"<<std::endl;

  test_vec[0] = 1;
  button_handler_.update((test_vec));
  std::cout<<"---"<<std::endl;

  test_vec[2] = 1;
  button_handler_.update((test_vec));
  std::cout<<"---"<<std::endl;


  test_vec[5] = 1;
  button_handler_.update((test_vec));
  std::cout<<"---"<<std::endl;


  test_vec[3] = 1;
  test_vec[5] = 0;
  button_handler_.update((test_vec));
//  std::cout<<"---"<<(ros::Time::now()-start).toSec()<<std::endl;

  return 0;
}
