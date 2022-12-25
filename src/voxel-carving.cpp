#include "crv/Application.h"

int main()
{    
    crv::Application::get().init();
    crv::Application::get().run();

    return 0;
}