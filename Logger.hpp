#pragma once
#include <Springhead.h>
#include <Windows.h>
#include <Framework/SprFWApp.h>
#include <Framework/SprFWConsoleDebugMonitor.h>
#include <vector>
#include <map>
#include <direct.h>

#include "conditions.hpp"

using namespace Spr;
class Logger {
public:
	Logger() {
		logFile = nullptr;
		SYSTEMTIME st;
		GetLocalTime(&st);
		char buf[256];
		sprintf_s(buf, "log_%04d%02d%02d_%02d%02d%02d/",
			st.wYear, st.wMonth, st.wDay,
			st.wHour, st.wMinute, st.wSecond);
		dir_name = buf;
		memset(&condition, 0, sizeof(Condition));
		memset(&data, 0, sizeof(LogData));
		count = 0;
		int res = _mkdir(dir_name.c_str());
	}

	Logger(std::string directory) {
		logFile = nullptr;
		dir_name = directory;
		if (dir_name.back() != '/' && dir_name.back() != '\\') {
			dir_name += "/";
		}
		int res = _mkdir(dir_name.c_str());
	}

	void saveSample() {
		fwrite(&data, sizeof(LogData), 1, logFile);
	}

	void open(std::string filename = "") {
		if (filename == "") {
			filename = std::to_string(count) + ".bin";
		}
		else {
			filename = std::to_string(count) + "_" + filename + ".bin";
		}
		count++;

		if (logFile) {
			close();
		}

		fopen_s(&logFile,(dir_name + filename).c_str(), "wb");

		if (!logFile) {
			throw std::runtime_error("Failed to open log file");
		}
		else {
			fwrite(&condition, sizeof(Condition), 1, logFile);
		}
	}

	void close() {
		if (logFile) {
			fclose(logFile);
			logFile = nullptr;
		}
	}

	Condition condition;

	struct LogData {
		unsigned long t;
		double load_pos[3];
		double pointer_pos[3];
		double grip_force;

		// Friction State
		double lugre_z[2];
		double lugre_dz[2];
		double lugre_v[2];
		double lugre_T;
		bool is_static_friction;

		double friction_force;
		double vibration_force;
		double mass;
	}data;

private:

	FILE* logFile;
	std::string dir_name;
	int count;
};