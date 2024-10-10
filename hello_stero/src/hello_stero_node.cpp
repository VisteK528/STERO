#include <cstdio>
#include <string>
#include <fstream>
#include <iostream>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::ifstream inFile("src/README.md");
  std::string name_and_surname;

  int i = 0;
  while(std::getline(inFile, name_and_surname) && i < 3){
    if(i == 1 || i == 2){
      std::cout<<name_and_surname<<std::endl;
    }
    ++i;
  }
  return 0;
}
