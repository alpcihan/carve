#include "crv/application/Application.h"

int main()
{    
    crv::Application::get().init();
    crv::Application::get().run();
    crv::Application::get().terminate();

    return 0;
}