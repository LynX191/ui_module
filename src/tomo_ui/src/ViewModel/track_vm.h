#ifndef TRACKVM_H
#define TRACKVM_H

#pragma once

#include <QImage>
#include <QObject>
#include <QQmlApplicationEngine>
#include <QQmlContext>

// My include
#include "../Model/image_model.h"
// end

class TrackVM : public QObject
{
	Q_OBJECT
	Q_PROPERTY(ImageModel* imageModel READ imageModel WRITE setImageModel NOTIFY imageModelChanged)

public:
	TrackVM(QStringList listDocName, QObject* parent = nullptr);
	~TrackVM();

public:
	ImageModel* imageModel();

	void UpdateImage(QImage image, int docIndex);
	void UpdateAllImage(QVector<QImage>& images);
	void resetColorBackgroundView(int docIndex);

	int trackID;
	int numDoc;

private:
	ImageModel* _imageModel;

	QVector<ImageViewFeature> imageViewFeatures;
	QHash<int, bool> stateChangedBackground;

	void changeColorBackgroundView(int docIndex);

Q_SIGNALS:
	void imageModelChanged();

public Q_SLOTS:
	void setImageModel(ImageModel* value);
};

#endif	// TRACKVM_H
