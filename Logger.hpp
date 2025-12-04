#pragma once
#include <Springhead.h>
#include <Windows.h>
#include <Framework/SprFWApp.h>
#include <Framework/SprFWConsoleDebugMonitor.h>
#include <vector>
#include <map>

using namespace Spr;
class Logger {
public:
	Logger() {
		logFile = nullptr;
		memset(&condition, 0, sizeof(Condition));
		memset(&data, 0, sizeof(LogData));
	}

	void saveSample() {
		fwrite(&data, sizeof(LogData), 1, logFile);
	}

	void open(std::string filename = "") {
		if (filename == "") {
			SYSTEMTIME st;
			GetLocalTime(&st);
			char buf[256];
			sprintf_s(buf, "log_%04d%02d%02d_%02d%02d%02d.bin",
				st.wYear, st.wMonth, st.wDay,
				st.wHour, st.wMinute, st.wSecond);
			filename = buf;
		}

		if (logFile) {
			close();
		}

		fopen_s(&logFile, filename.c_str(), "wb");

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

	struct Condition {
		unsigned int friction_model; // 1: Lugre, 0: Coulomb
		union {
			struct {
				double sigma0;
				double sigma1;
				double sigma2;
				double A;
				double B;
				double C;
			}lugre;
			struct {
				double mu0;
				double mu;
			}coulomb;
		};
		double mass;
	}condition;

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

	}data;

private:

	FILE* logFile;
};