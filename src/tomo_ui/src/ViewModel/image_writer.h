#ifndef IMAGE_WRITER_H
#define IMAGE_WRITER_H

#pragma once

#include <QImage>
#include <QPainter>
#include <QQuickItem>
#include <QQuickPaintedItem>
// My include

// end

class ImageWriter : public QQuickPaintedItem
{
	Q_OBJECT

	// property zone
	Q_PROPERTY(QImage ui_image MEMBER image WRITE setImage)
	Q_PROPERTY(double ui_width READ ui_width WRITE setUi_width NOTIFY ui_widthChanged)
	Q_PROPERTY(double ui_height READ ui_height WRITE setUi_height NOTIFY ui_heightChanged)
	Q_PROPERTY(double ui_top READ ui_top WRITE setUi_top NOTIFY ui_topChanged)
	Q_PROPERTY(double ui_left READ ui_left WRITE setUi_left NOTIFY ui_leftChanged)
	Q_PROPERTY(double roiX READ roiX WRITE setRoiX NOTIFY roiXChanged)
	Q_PROPERTY(double roiY READ roiY WRITE setRoiY NOTIFY roiYChanged)
	Q_PROPERTY(double roiWidth READ roiWidth WRITE setRoiWidth NOTIFY roiWidthChanged)
	Q_PROPERTY(double roiHeight READ roiHeight WRITE setRoiHeight NOTIFY roiHeightChanged)
	// end

public:
	ImageWriter(QQuickItem* parent = nullptr);
	~ImageWriter();

public:
private:
	QImage image;

	QImage temp_image;
	//  property use in qml

	double ui_width();
	double ui_height();
	double ui_top();
	double ui_left();
	double roiX();
	double roiY();
	double roiWidth();
	double roiHeight();

	// end

	// variable property

	double _ui_width;
	double _ui_height;
	double _ui_top;
	double _ui_left;
	double _roiX;
	double _roiY;
	double _roiWidth;
	double _roiHeight;

	// end

	double oldUi_width;
	double oldUi_height;

	void paint(QPainter* painter) override;
	void setImage(QImage& image);

Q_SIGNALS:
	// Notify property changed
	void ui_widthChanged();
	void ui_heightChanged();
	void ui_topChanged();
	void ui_leftChanged();
	void roiXChanged();
	void roiYChanged();
	void roiWidthChanged();
	void roiHeightChanged();

	// end

	void updateSourceImage();
	void requestZoomFit();

public Q_SLOTS:
	// Set property zone
	void setUi_width(double);
	void setUi_height(double);
	void setUi_top(double);
	void setUi_left(double);
	void setRoiX(double);
	void setRoiY(double);
	void setRoiWidth(double);
	void setRoiHeight(double);

	// end
	void resetImage();
	void requestStretch(int, int);
	void clearTmpImage();
};

#endif	// IMAGE_WRITER_H
