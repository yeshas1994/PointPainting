#pragma once
#include <vector>

struct InferResult
{
	float width() const {
		return x2 - x1;
	}

	float height() const {
		return y2 - y1;
	}

	float x1 = 0;
	float y1 = 0;
	float x2 = 0;
	float y2 = 0;
	int classIndex = -1;
	float score = 0;
};
//! Meta info per frame
struct ImageMeta
{
	std::vector<InferResult> Inferred;
};
