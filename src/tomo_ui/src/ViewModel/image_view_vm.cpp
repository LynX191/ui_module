#include "image_view_vm.h"
#include <QDebug>
#include <ros/ros.h>

ImageViewVM::ImageViewVM(QObject* parent) : QObject(parent)
{
	_averageIntensity = "black";
	_viewMode		  = 0;
	_noSignal		  = true;
}

ImageViewVM::~ImageViewVM()
{
}

QString ImageViewVM::averageIntensity()
{
	return _averageIntensity;
}

QImage ImageViewVM::image()
{
	return _image;
}

int ImageViewVM::viewMode()
{
	return _viewMode;
}

bool ImageViewVM::noSignal()
{
	return _noSignal;
}

void ImageViewVM::setImage(QImage& image, DocViewMode viewMode, bool isBlankImage)
{
	if(!isBlankImage && _noSignal) {
		_noSignal = false;
		Q_EMIT noSignalChanged();
	}
	setViewMode((int) viewMode);
	_image = image;
		Q_EMIT imageChanged();
}
// property

int ImageViewVM::imageWidth()
{
	return _image.width();
}

int ImageViewVM::imageHeight()
{
	return _image.height();
}

void ImageViewVM::positionMouseHoverInImageUI(float posX, float posY)
{
	if((posX > _image.width()) || posX < 0 || posY > _image.height() || posY < 0) {
		return;
	}
	ImageColor_PositionInfo imageColor_PositionInfo;
	if(_image.isNull()) {
		imageColor_PositionInfo.red	  = 0;
		imageColor_PositionInfo.green = 0;
		imageColor_PositionInfo.blue  = 0;
		imageColor_PositionInfo.gray  = 0;
		imageColor_PositionInfo.posX  = int(posX);
		imageColor_PositionInfo.posY  = int(posY);
		Q_EMIT sendColor_PosValueToTrack_Signals(imageColor_PositionInfo);
	}
	else {
		try {
			QColor color				  = _image.pixelColor(int(posX), int(posY));
			int gray					  = int(color.red() * 0.299 + color.green() * 0.587 + color.blue() * 0.114);
			imageColor_PositionInfo.red	  = color.red();
			imageColor_PositionInfo.green = color.green();
			imageColor_PositionInfo.blue  = color.blue();
			imageColor_PositionInfo.gray  = gray;
			imageColor_PositionInfo.posX  = int(posX);
			imageColor_PositionInfo.posY  = int(posY);
			Q_EMIT sendColor_PosValueToTrack_Signals(imageColor_PositionInfo);
		}
		catch(const std::exception& e) {
			imageColor_PositionInfo.red	  = 0;
			imageColor_PositionInfo.green = 0;
			imageColor_PositionInfo.blue  = 0;
			imageColor_PositionInfo.gray  = 0;
			imageColor_PositionInfo.posX  = 0;
			imageColor_PositionInfo.posY  = 0;
			Q_EMIT sendColor_PosValueToTrack_Signals(imageColor_PositionInfo);
		}
	}
}

void ImageViewVM::setViewMode(int iViewMode)
{
	if(_viewMode != iViewMode) {
		_viewMode = iViewMode;
		Q_EMIT viewModeChanged();
	}
}

void ImageViewVM::setAverageIntensity(QString value)
{
	if(_averageIntensity != value) {
		_averageIntensity = value;
		Q_EMIT averageIntensityChanged();
	}
}

// end property
