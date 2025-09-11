#pragma once
#include <Springhead.h>
#include <Windows.h>
#include <Framework/SprFWApp.h>
#include <Framework/SprFWConsoleDebugMonitor.h>
#include <vector>
#include <map>
#include <fstream>

using namespace Spr;



class Logger {
public:
	Logger(std::string filename, unsigned int _maxSample = 1000) : logfile(filename), maxSample(_maxSample), buffer(_maxSample), newLabel(false) {
	}

	void addData(std::string label, double value, double max = 1.0, double min = 2.0) {
		auto s = index.find(label);
		if (s == index.end()) {
			// new data
			int i = index.size();
			index[label] = i;
			labels.push_back(label);
			sample.push_back(0);
			setRange(label, max, min);
			newLabel = true;
		}
		sample[index[label]] = value;

	}

	void writeData() {
		if (newLabel) {
			saveLabel();
			newLabel = false;
		}
		buffer.Write(sample);
	}

	void setRange(std::string label, double max, double min) {
		rangeMax[label] = max;
		rangeMin[label] = min;
	}

	void drawGraph(GRRenderIf* r, unsigned int length = 2000) {
		/*
		r->PushProjectionMatrix();
		r->SetProjectionMatrix(Affinef::OrthoGL(Vec3f(0, 0, 10), Vec2f(2, 2), -10, 10));
		Affinef af;
		af.Ey() *= -1;
		af.Ex() *= -1;
		r->SetViewMatrix(af);
		r->SetModelMatrix(Affinef());
		r->SetDepthTest(false);
		r->SetLighting(false);

		const float dx = 0.001f;
		const float h = 0.25f; // Height of the graph
		float y0 = -1 + h;
		int sampleLength = 0;
		if (samples.size())
			sampleLength = samples.begin()->second.size(); // Size of first data column
		if (sampleLength >= 10) {
			int t0 = 0;
			if (sampleLength < length) {
				length = sampleLength;
			}
			else {
				t0 = sampleLength - length;
			}


			float graphYoffset = -0.7;
			for (auto itr = samples.begin(); itr != samples.end(); ++itr) {
				std::string label = itr->first;
				std::vector<double> value = itr->second;

				// Calculate scale of graph
				float min, max;
				min = rangeMin[label];
				max = rangeMax[label];
				if (min > max) {
					// Auto Range
					min = *min_element(value.end() - length, value.end());
					max = *max_element(value.end() - length, value.end());
				}
				else {
					// Fixed Range

				}
				float scale = h / (max - min);

				int tStart = 0;

				// Draw axes
				float yAxis = -(max + min) / 2.0 * scale + graphYoffset;
				r->DrawLine(Vec3f(-1.0, yAxis, 0.0), Vec3f(1.0, yAxis, 0.0));	// horizontal
				r->DrawLine(Vec3f(-0.99, graphYoffset + h * 0.55, 0), Vec3f(-0.99, graphYoffset - h * 0.55, 0.0)); // virtical
				r->DrawLine(Vec3f(-0.999, graphYoffset + h / 2.0, 0), Vec3f(-0.98, graphYoffset + h / 2.0, 0.0)); // max
				r->DrawFont(Vec3f(-0.98, graphYoffset + h / 1.7, 0), std::to_string(max));
				r->DrawLine(Vec3f(-0.999, graphYoffset - h / 2.0, 0), Vec3f(-0.98, graphYoffset - h / 2.0, 0.0)); // min
				r->DrawFont(Vec3f(-0.98, graphYoffset - h / 1.7, 0), std::to_string(min));
				// Legend
				r->DrawFont(Vec3f(-0.95, graphYoffset - h / 1.5, 0), label);
				for (int t = 0; t < length - 2; t++) {
					float x = -0.99;
					r->DrawLine(Vec3f(x + t * dx, value[t + t0] * scale + yAxis, 0), Vec3f(x + (t + 1) * dx, value[t + t0 + 1] * scale + yAxis, 0));
				}
				graphYoffset += h * 1.5;
			}

			//r->DrawFont(Vec3f(-1, 0.95f, 0), os.str());
		}
		r->PopProjectionMatrix();
		*/
	}

	void startRecord() {
	}

	void saveRecord() {
		std::vector<double> rec;
		while (buffer.Read(rec)) {
			logfile << std::endl;
			for (int i = 0; i < rec.size(); i++) {
				logfile << std::scientific << rec[i] << ',';
			}
		}
	}

	void saveLabel() {
		logfile << std::endl;;
		for (auto itr = labels.begin(); itr != labels.end(); ++itr) {
			logfile <<  *itr << ',';
		}
	}

private:
	const unsigned int maxSample;

	std::map<std::string, int> index;
	std::vector<std::string> labels;
	std::vector<double> sample;
	std::map<std::string, double > rangeMax, rangeMin;
	std::ofstream logfile;
	bool newLabel;
	RingBuffer<std::vector<double>> buffer;
};