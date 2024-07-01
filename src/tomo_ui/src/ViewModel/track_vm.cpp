#include "track_vm.h"

TrackVM::TrackVM(QStringList listDocName, QObject* parent) : QObject{parent}
{
	// create track
	numDoc	   = listDocName.size();
	QImage tmp = QImage(1024, 1024, QImage::Format_RGB888);
	for(int i = 0; i < numDoc; i++) {
		ImageViewFeature imageViewFeature;
		imageViewFeature.name		 = listDocName[i];
		imageViewFeature.imageViewVM = new ImageViewVM();
		imageViewFeature.imageViewVM->setImage(tmp, DocViewMode::Original, true);
		imageViewFeatures.append(imageViewFeature);
		stateChangedBackground[i] = false;
	}
	_imageModel = new ImageModel();
	_imageModel->createData(imageViewFeatures);

	// create connect
}

TrackVM::~TrackVM()
{
	delete _imageModel;
}

// Property
ImageModel* TrackVM::imageModel()
{
	return _imageModel;
}

void TrackVM::setImageModel(ImageModel* value)
{
	_imageModel = value;
	Q_EMIT imageModelChanged();
}
// end property

// void TrackVM::receivePosValueFromImageVM_Slots(int x, int y)
// {
//     Q_EMIT sendPosValueToOptispecVM_Signals(x, y);
// }

void TrackVM::UpdateImage(QImage image, int docIndex)
{
	if(docIndex >= imageViewFeatures.size())
		return;

	imageViewFeatures.at(docIndex).imageViewVM->setImage(image);
	changeColorBackgroundView(docIndex);
}

void TrackVM::UpdateAllImage(QVector<QImage>& images)
{
	if(images.size() != imageViewFeatures.size())
		return;

	for(int i = 0; i < images.size(); i++) {
		imageViewFeatures.at(i).imageViewVM->setImage(images[i]);
		changeColorBackgroundView(i);
	}
}

void TrackVM::resetColorBackgroundView(int docIndex)
{
	if(docIndex >= imageViewFeatures.size())
		return;
	stateChangedBackground[docIndex] = false;
}

void TrackVM::changeColorBackgroundView(int docIndex)
{
	if(docIndex >= imageViewFeatures.size())
		return;

	if(stateChangedBackground.value(docIndex))
		return;

	stateChangedBackground[docIndex] = true;

	QString averageIntensity;
	QImage tempImg	   = imageViewFeatures.at(docIndex).imageViewVM->image();
	int a_r			   = 0;
	int a_g			   = 0;
	int a_b			   = 0;
	int width		   = tempImg.width();
	int height		   = tempImg.height();
	int oneFifthHeight = height / 5;
	int totalPixel	   = 0;
	for(int x = width - 2; x < width; x++) {
		for(int y = oneFifthHeight; y < height - oneFifthHeight; y++) {
			QColor color = tempImg.pixelColor(x, y);
			a_r += color.red();
			a_g += color.green();
			a_b += color.blue();
			totalPixel++;
		}
	}
	a_r				 = int(a_r / totalPixel);
	a_g				 = int(a_g / totalPixel);
	a_b				 = int(a_b / totalPixel);
	averageIntensity = QColor(a_r, a_g, a_b).name();
	imageViewFeatures.at(docIndex).imageViewVM->setAverageIntensity(averageIntensity);
}
