#pragma once

#define DllExport __declspec(dllexport)
#define CDllExport extern "C" __declspec(dllexport)

namespace GridWorld
{
    class EntityManager;

    DllExport EntityManager& create_test_em();

    DllExport void run_test(EntityManager& em);
}