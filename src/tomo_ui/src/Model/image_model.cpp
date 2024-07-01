#include "image_model.h"

ImageModel::ImageModel(QObject* parent) : QAbstractListModel(parent)
{

}

ImageModel::~ImageModel()
{

}

void ImageModel::addData(const ImageViewFeature& entry)
{
	beginInsertRows(QModelIndex(), rowCount(), rowCount());
	_model << entry;
	endInsertRows();
}

int ImageModel::rowCount(const QModelIndex& parent) const
{
	parent.isValid();
	return _model.count();
}

QVariant ImageModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid() || index.row() < 0 || index.row() >= _model.count())
		return QVariant();
	ImageViewFeature modelEntry = _model[index.row()];
	if(role == nameRole) {
		return modelEntry.name;
	}
	if(role == imageViewVMRole) {
		return QVariant::fromValue(modelEntry.imageViewVM);
	}
	return QVariant();
}

// roleNames() method for QAbstractListModel:
QHash<int, QByteArray> ImageModel::roleNames() const
{
	QHash<int, QByteArray> roles;
	roles[nameRole]		   = "name";
	roles[imageViewVMRole] = "imageViewVM";
	return roles;
}

void ImageModel::editInputData(int row, const QVariant& value, int role)
{
	QModelIndex index = this->index(row, 0);
	if(!index.isValid() || index.row() < 0 || index.row() >= _model.count())
		return;
	if(role == nameRole) {
		_model[index.row()].name = value.toString();
	}
	if(role == imageViewVMRole) {
        _model[index.row()].imageViewVM = value.value<ImageViewVM*>();
	}
	Q_EMIT dataChanged(index, index, {role});
}

void ImageModel::createData(QVector<ImageViewFeature> data)
{
	//    ImageViewFeature imageViewFeature;// = new ImageViewFeature[data.size()];
	for(int i = 0; i < data.size(); i++) {
		ImageViewFeature imageViewFeature;
		imageViewFeature.name		 = data[i].name;
		imageViewFeature.imageViewVM = data[i].imageViewVM;
		_model << imageViewFeature;
	}
}
