#ifndef IMAGEVIEWVM_H
#define IMAGEVIEWVM_H

#pragma once

#include <QObject>
#include <QPainter>

// My include
#include "../Script/Define/struct_def.h"
// end

class ImageViewVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(QImage image MEMBER _image READ image WRITE setImage NOTIFY imageChanged)
	Q_PROPERTY(int viewMode READ viewMode WRITE setViewMode NOTIFY viewModeChanged)
	Q_PROPERTY(QString averageIntensity READ averageIntensity WRITE setAverageIntensity NOTIFY averageIntensityChanged)

	Q_PROPERTY(bool noSignal READ noSignal NOTIFY noSignalChanged)

public:
	ImageViewVM(QObject* parent = nullptr);
	~ImageViewVM();

public:
	// property
	QString averageIntensity();
	QImage image();
	int viewMode();
	bool noSignal();
	// end

private:
	// Property
	QImage _image;
	int _viewMode;
	QString _averageIntensity;
	bool _noSignal;
	// End property

Q_SIGNALS:
	// Property
	void imageChanged();
	void viewModeChanged();
	void averageIntensityChanged();
	void noSignalChanged();
	// End property
	void sendColor_PosValueToTrack_Signals(ImageColor_PositionInfo&);

public Q_SLOTS:
	// Property
	void setImage(QImage& image, DocViewMode viewMode = DocViewMode::Original, bool isBlankImage = false);
	void setViewMode(int);
	void setAverageIntensity(QString);
	// End property
	int imageWidth();
	int imageHeight();
	void positionMouseHoverInImageUI(float, float);
};

#endif	// IMAGEVIEWVM_H
