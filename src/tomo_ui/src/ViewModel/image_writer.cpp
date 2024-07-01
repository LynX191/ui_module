#include "image_writer.h"
#include <ros/ros.h>

ImageWriter::ImageWriter(QQuickItem* parent) : QQuickPaintedItem(parent), image{}
{
	image = QImage();
	temp_image.fill(Qt::black);
	_roiX		 = 0;
	_roiY		 = 0;
	_roiWidth	 = 100;
	_roiHeight	 = 100;
	oldUi_width	 = 0;
	oldUi_height = 0;
}

ImageWriter::~ImageWriter()
{
}

// Get property
double ImageWriter::ui_width()
{
	return _ui_width;
}

double ImageWriter::ui_height()
{
	return _ui_height;
}

double ImageWriter::ui_top()
{
	return _ui_top;
}

double ImageWriter::ui_left()
{
	return _ui_left;
}

double ImageWriter::roiX()
{
	return _roiX;
}

double ImageWriter::roiY()
{
	return _roiY;
}

double ImageWriter::roiWidth()
{
	return _roiWidth;
}

double ImageWriter::roiHeight()
{
	return _roiHeight;
}

// end

void ImageWriter::paint(QPainter* painter)
{
	QRectF bouding_rect = boundingRect();
	QRect ROI(_roiX, _roiY, _roiWidth, _roiHeight);
	QImage crop = image.copy(ROI);

	if(crop.isNull() || image.isNull()) {
		return;
	}
	QImage scaled = crop.scaled(bouding_rect.width(), bouding_rect.height(), Qt::KeepAspectRatio);

	if((oldUi_width != _ui_width) || (oldUi_height != _ui_height)) {
		oldUi_width	 = _ui_width;
		oldUi_height = _ui_height;
		Q_EMIT requestZoomFit();
	}
	painter->drawImage(QPoint(0, 0), scaled);
}

// Set property
void ImageWriter::setImage(QImage& _image)
{

	if(_image.isNull())
		image = temp_image;
	else {
		temp_image = _image;
		image	   = _image;
	}
	setUi_width(image.size().width());
	setUi_height(image.size().height());
	update();
	Q_EMIT updateSourceImage();
}

void ImageWriter::setUi_width(double value)
{
	if(_ui_width != value) {
		_ui_width = value;
		Q_EMIT ui_widthChanged();
	}
}

void ImageWriter::setUi_height(double value)
{
	if(_ui_height != value) {
		_ui_height = value;
		Q_EMIT ui_heightChanged();
	}
}

void ImageWriter::setUi_top(double value)
{
	if(_ui_top != value) {
		_ui_top = value;
		Q_EMIT ui_topChanged();
	}
}

void ImageWriter::setUi_left(double value)
{
	if(_ui_left != value) {
		_ui_left = value;
		Q_EMIT ui_leftChanged();
	}
}

void ImageWriter::setRoiX(double value)
{
	if(_roiX != value) {
		_roiX = value;
		Q_EMIT roiXChanged();
	}
}

void ImageWriter::setRoiY(double value)
{
	if(_roiY != value) {
		_roiY = value;
		Q_EMIT roiYChanged();
	}
}

void ImageWriter::setRoiWidth(double value)
{
	if(_roiWidth != value) {
		_roiWidth = value;
		Q_EMIT roiWidthChanged();
	}
}

void ImageWriter::setRoiHeight(double value)
{
	if(_roiHeight != value) {
		_roiHeight = value;
		Q_EMIT roiHeightChanged();
	}
}

void ImageWriter::resetImage()
{
	return;
}

void ImageWriter::requestStretch(int docWidth, int docHeight)
{
	double scaled = docWidth / _ui_width;
	if(docHeight / _ui_height > scaled) {
		scaled = docHeight / _ui_height;
	}
	image = image.scaled(docWidth / scaled, docHeight / scaled);
	setUi_width(image.size().width());
	setUi_height(image.size().height());
	update();
}

void ImageWriter::clearTmpImage()
{
	temp_image = QImage(0, 0, QImage::Format_RGB888);
	temp_image.fill(Qt::black);
}

// end
