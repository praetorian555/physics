#include "catch2-helper.hpp"

int main(int argc, char* argv[])
{
    Catch::Session session;
    const int return_code = session.applyCommandLine(argc, argv);
    if (return_code != 0)
    {
        return return_code;
    }
    return session.run();
}