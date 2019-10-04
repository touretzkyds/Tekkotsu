#include "PGMImg.h"
#include "../SIFTDatabase/keypoint.h"
#include "../SIFTDatabase/keypointpair.h"
#include <cstring>
#include <iostream>
#include <cmath>
#include <cstdlib>

void PGMImg::setPixel(int x, int y, unsigned char color){
	int loc = x+width*y;
	if (loc >= 0 && loc < width*height) pixels[x+width*y] = color;
}

unsigned char PGMImg::getPixel(int x, int y){
	int loc = x+width*y;
	if (loc >= 0 && loc < width*height) return pixels[x+width*y];
	return 0;
}

PGMImg::PGMImg() : width(0), height(0), base(0), pixels(NULL), filename(), id(0) {}

void PGMImg::fromFile(std::string imageFilename){
	pixels = NULL;
	
	std::ifstream infile(imageFilename.c_str());
	
	std::string input;
	
	infile >> input;
	
	if (input.compare("P2")){
		std::cout << imageFilename << " is not ASCII PGM file\n";
	}else{
		infile >> input;
		while (input[0] == '#'){
			getline(infile, input);
			infile >> input;
		}
		width = atoi(input.c_str());
		infile >> height;
		infile >> base;
		
		if (base > 255){
			std::cout << "Unable to process non 8-bit PGM file\n";
		}else{
			pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
			bzero(pixels, sizeof(pixels));
			
			int temp;
			for (int i = 0; i < height; i++){
				for (int j = 0; j < width; j++){
					infile >> temp;
					setPixel(j, i, (unsigned char)temp);
				}
			}
		}
	}
	
	infile.close();
	
	filename = imageFilename;
}

PGMImg::PGMImg(PGMImg* img1, PGMImg* img2, std::vector<keypointPair*>* matches)
  : width(0), height(0), base(0), pixels(NULL), filename(), id(0){
	if (img1->pixels && img2->pixels){
//		height = (img1->height > img2->height) ? img1->height : img2->height;
//		width = img1->width  + img2->width;
		height = img1->height + img2->height;
		width = (img1->width > img2->width) ? img1->width : img2->width;
		pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
		//bzero(pixels, sizeof(pixels));
		for (int i = 0; i < height; i++){
			for (int j = 0; j < width; j++){
				setPixel(j, i, 0);
			}
		}
		base = 255;
		
		for (int i = 0; i < (int)(*matches).size(); i++){
			img1->drawKeypoint((*matches)[i]->getKey2());
			img2->drawKeypoint((*matches)[i]->getKey1());
		}
		
		for (int i = 0; i < img1->height; i++){
			for (int j = 0; j < img1->width; j++){
				setPixel(j, i, img1->getPixel(j,i));
			}
		}
		for (int i = 0; i < img2->height; i++){
			for (int j = 0; j < img2->width; j++){
				setPixel(j, i+img1->height, img2->getPixel(j,i));
			}
		}
		
		for (int i = 0; i < (int)(*matches).size(); i++){
			drawLine((int)((*matches)[i]->getKey2()->imageX), (int)((*matches)[i]->getKey2()->imageY), (int)((*matches)[i]->getKey1()->imageX), (int)((*matches)[i]->getKey1()->imageY) + img1->height, 127);
		}
	}
}

PGMImg::~PGMImg(){
// 	cout << id << " " << (unsigned int)pixels << endl;
// 	cout << "checkpoint11\n";
	if (pixels) free(pixels);
// 	cout << "checkpoint12\n";
	pixels = NULL;
// 	cout << "checkpoint13\n";
}

int PGMImg::getHeight(){
	return height;
}

int PGMImg::getWidth(){
	return width;
}

void PGMImg::toFile(std::string imageFilename){
	std::ofstream outfile(imageFilename.c_str());
//	std::cout << outfile.good() << outfile.bad() << outfile.fail() << std::endl;
	
	outfile << "P2\n";
	outfile << width << " " << height << std::endl;
	outfile << base << std::endl;
	
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			char intString[1024];
			bzero(intString, sizeof(intString));
			outfile << (int)getPixel(j, i) << " ";
		}
		outfile << std::endl;
	}
	
	outfile.close();

}

void PGMImg::drawKeypoint(keypoint* key){
	double x, y, scale, orientation;
	x = key->imageX;
	y = key->imageY;
	scale = key->imageScale;
	orientation = key->imageOrientation;
	double radius = 2.0 * scale;
	double centerX = x;
	double centerY = y;
	double x1 = (x - (radius * std::cos(orientation + (M_PI_4))));
	double x2 = (x - (radius * std::cos(orientation + (M_PI_2 + M_PI_4))));
	double x3 = (x - (radius * std::cos(orientation - (M_PI_2 + M_PI_4))));
	double x4 = (x - (radius * std::cos(orientation - (M_PI_4))));
	double y1 = (y - (radius * std::sin(orientation + (M_PI_4))));
	double y2 = (y - (radius * std::sin(orientation + (M_PI_2 + M_PI_4))));
	double y3 = (y - (radius * std::sin(orientation - (M_PI_2 + M_PI_4))));
	double y4 = (y - (radius * std::sin(orientation - (M_PI_4))));
	
	drawLine((int)(x1), (int)(y1), (int)(x2), (int)(y2), 0);
	drawLine((int)(x2), (int)(y2), (int)(x3), (int)(y3), 0);
	drawLine((int)(x3), (int)(y3), (int)(x4), (int)(y4), 0);
	drawLine((int)(x4), (int)(y4), (int)(x1), (int)(y1), 0);
	drawLine((int)((x1 + x4)/2.0), (int)((y1 + y4)/2.0), (int)(centerX), (int)(centerY), 0);
	
//	cout << "(" << x << "," << y << ")\n";
//	cout << "(" << (int)x1 << "," << (int)y1 << ")\n";
//	cout << "(" << (int)x2 << "," << (int)y2 << ")\n";
//	cout << "(" << (int)x3 << "," << (int)y3 << ")\n";
//	cout << "(" << (int)x4 << "," << (int)y4 << ")\n";
//	cout << "(" << (int)centerX << "," << (int)centerY << ")\n";
}

void PGMImg::drawKeypoints(std::vector<keypoint*> keys){
	for (int i = 0; i < (int)keys.size(); i++){
		drawKeypoint(keys[i]);
	}
}

void PGMImg::drawLine(int x1, int y1, int x2, int y2, unsigned char color){
	int dx = x2 - x1;
	dx = (dx < 0) ? -dx : dx;
	int dy = y2 - y1;
	dy = (dy < 0) ? -dy : dy;
	
//	cout << "|" << x1 << "-" << x2 << "|=" << dx
//	     << "|" << y1 << "-" << y2 << "|=" << dy
//	     << endl;
	
	
	if (dx < dy){
		if (y1 > y2){
			int temp;
			temp = x1;
			x1 = x2;
			x2 = temp;
			temp = y1;
			y1 = y2;
			y2 = temp;
		}
		double deltax = (double)(x2-x1) / (double)(y2-y1);
		
//		cout << "(" << (int)x1 << "," << (int)y1 << ")\n";
//		cout << "(" << (int)x2 << "," << (int)y2 << ")\n";
//		cout << deltax << endl;
		
		double x = x1 + 0.5;
		int y = y1;
		for (; y <= y2; y++){
			setPixel((int)x, y, color);
			x += deltax;
		}
	}else{
		if (x1 > x2){
			int temp;
			temp = x1;
			x1 = x2;
			x2 = temp;
			temp = y1;
			y1 = y2;
			y2 = temp;
		}
		double deltay = (double)(y2-y1) / (double)(x2-x1);
		double y = y1 + 0.5;
		int x = x1;
		for (; x <= x2; x++){
			setPixel(x, (int)y, color);
			y += deltay;
		}
	}
}

void PGMImg::translate(int x, int y, unsigned char color){
	int minx, miny, maxx, maxy, newX, newY;
	if (x < 0){
		minx = x;
		maxx = width - 1;
		newX = 0;
	}else{
		minx = 0;
		maxx = width + x - 1;
		newX = x;
	}
	if (y < 0){
		miny = y;
		maxy = height - 1;
		newY = 0;
	}else{
		miny = 0;
		maxy = height + y - 1;
		newY = y;
	}
	
	int oldWidth  = width;
	int oldHeight = height;
	
	width  = maxx - minx + 1;
	height = maxy - miny + 1;
	
	unsigned char* oldPixels = pixels;
	pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			setPixel(j, i, color);
		}
	}
	
	for (int i = 0; i < oldHeight; i++){
		int tempnewX = newX;
		for (int j = 0; j < oldWidth; j++){
			setPixel(tempnewX, newY, oldPixels[j+oldWidth*i]);
			tempnewX++;
		}
		newY++;
	}
	
	free(oldPixels);
}

void PGMImg::rotate(double s, unsigned char color){
	double centerX, centerY;
	
	centerX = 0.5 * (double)(width);
	centerY = 0.5 * (double)(height);
	
	unsigned char* oldPixels = pixels;
	pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			setPixel(j, i, color);
		}
	}
	
	for (int i = 0; i < height; i++){
		double adjY = centerY - i;
		for (int j = 0; j < width; j++){
			double adjX = j - centerX;
			
			if (i == centerY && j == centerX){
				setPixel(j, i, oldPixels[(width/2)+width*(height/2)]);
				continue;
			}
			
			double h = sqrt(adjX * adjX + adjY * adjY);
			double alpha = acos(adjX / h);
			if (adjY < 0.0) alpha = acos(-adjX/h) + M_PI;
			
			double adjXPrime = h * cos(s + alpha);
			double adjYPrime = h * sin(s + alpha);
			
			double xPrime = adjXPrime + centerX;
			double yPrime = centerY - adjYPrime;
			
			int intX = (int)xPrime;
			int intY = (int)yPrime;
			double px = xPrime - (double)intX;
			double py = yPrime - (double)intY;
			
			double sum = 0.0;
			double base1 = 0.0;
			
			int index;
			
			index = intX + width * intY;
			if (index >= 0 && index < width*height){
				sum  += (1-px) * (1-py) * oldPixels[index];
				base1 += (1-px) * (1-py);
			}
			index = (intX+1) + width * intY;
			if (index >= 0 && index < width*height){
				sum  += px     * (1-py) * oldPixels[index];
				base1 += px     * (1-py);
			}
			index = intX + width * (intY+1);
			if (index >= 0 && index < width*height){
				sum  += (1-px) * py     * oldPixels[index];
				base1 += (1-px) * py;
			}
			index = (intX+1) + width * (intY+1);
			if (index >= 0 && index < width*height){
				sum  += px     * py     * oldPixels[index];
				base1 += px     * py;
			}
			
//			if ((unsigned char)(sum/base1+0.5) != oldPixels[j+i*width]) cout << "(" << j << "," << i << ") --> (" << xPrime << "," << yPrime << ") " << px << "," << py << " " << sum << "," << base1 << "," << (sum/base1) << "," << (int)((unsigned char)(sum/base1 +0.5)) << " " << (int)(oldPixels[j+i*width]) << "\n";
			
			if (base1 != 0.0){
				setPixel(j, i, (unsigned char)(sum/base1+0.5));
			}
//			setPixel(j, i, (unsigned char)(
//							   px     * py     * oldPixels[smallX + width * smallY]
//							 + px     * (1-py) * oldPixels[smallX + width * largeY]
//							 + (1-px) * py     * oldPixels[largeX + width * smallY]
//							 + (1-px) * (1-py) * oldPixels[largeX + width * largeY]
//							));
		}
		
	}
	
}

void PGMImg::doubleSize(double s, unsigned char color){
	int oldWidth  = width;
//	int oldHeight = height;
	
	int scaleFactor = 1;
	for (int i = 0; i < (int)s; i++){
		scaleFactor*= 2;
	}
	
	width  *= scaleFactor;
	height *= scaleFactor;
	
	unsigned char* oldPixels = pixels;
	pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			setPixel(j, i, oldPixels[(j/scaleFactor)+oldWidth*(i/scaleFactor)]);
		}
	}
	free(oldPixels);
}

void PGMImg::halfSize(double s, unsigned char color){
	int oldWidth  = width;
//	int oldHeight = height;
	
	int scaleFactor = 1;
	for (int i = 0; i < (int)s; i++){
		scaleFactor*= 2;
	}
	
	width  /= scaleFactor;
	height /= scaleFactor;
	
	unsigned char* oldPixels = pixels;
	pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			int sum = 0;
			int starti = i * scaleFactor;
			int startj = j * scaleFactor;
			for (int ii = 0; ii < scaleFactor; ii++){
				for (int jj = 0; jj < scaleFactor; jj++){
					sum += (int)(oldPixels[(startj+jj)+oldWidth*(starti+ii)]);
				}
			}
			setPixel(j, i, (unsigned char)(sum/(scaleFactor*scaleFactor)));
		}
	}
	free(oldPixels);
}

void PGMImg::cropImage(int startx, int starty, int endx, int endy, unsigned char color){
	int oldWidth  = width;
	int oldHeight = height;
	
	width  = endx - startx + 1;
	height = endy - starty + 1;
	
	unsigned char* oldPixels = pixels;
	pixels = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			setPixel(j, i, color);
		}
	}
	
	int visibleStartX = (startx > 0) ? startx : 0;
	int visibleStartY = (starty > 0) ? starty : 0;

	int visibleEndX = (endx > oldWidth  - 1) ? oldWidth  - 1 : endx;
	int visibleEndY = (endy > oldHeight - 1) ? oldHeight - 1 : endy;
	
	for (int i = visibleStartY; i <= visibleEndY; i++){
		for (int j = visibleStartX; j <= visibleEndX; j++){
//			cout << "Setting pixel (" << j << "," << i << ")\n";
			setPixel(j - startx, i - starty, oldPixels[j+oldWidth*i]);
		}
	}
	
	free(oldPixels);
	
}
