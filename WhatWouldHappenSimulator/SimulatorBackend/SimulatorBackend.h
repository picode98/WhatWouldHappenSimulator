#pragma once

#ifdef SIMULATORBACKEND_EXPORTS
	#define SIMULATORBACKEND_DECLSPEC __declspec(dllexport)
#else
	#define SIMULATORBACKEND_DECLSPEC __declspec(dllimport)
#endif

extern "C" SIMULATORBACKEND_DECLSPEC void test();