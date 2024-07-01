#ifndef IMAGEMODEL_H
#define IMAGEMODEL_H

#include <QAbstractListModel>

// My include
#include "../ViewModel/image_view_vm.h"
// end

struct ImageViewFeature
{
	QString name;
    ImageViewVM* imageViewVM;
};

class ImageModel : public QAbstractListModel
{
public:
	explicit ImageModel(QObject* parent = 0);
	~ImageModel();
	// enum DataRoles for QAbstractListModel:
	enum DataRoles
	{
		nameRole		= Qt::UserRole + 1,
		imageViewVMRole = Qt::UserRole + 2
	};
	// addData() method for QAbstractListModel:
	void addData(const ImageViewFeature& entry);
	// rowCount() method for QAbstractListModel:
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	// data() required for QAbstractListModel:
	QVariant data(const QModelIndex& index, int role) const;
	// roleNames() method for QAbstractListModel:
	QHash<int, QByteArray> roleNames() const;
	void editInputData(int row, const QVariant& value, int role = Qt::EditRole);
	void createData(QVector<ImageViewFeature> data);
	QVector<ImageViewFeature> _model;
};

#endif	// IMAGEMODEL_H
