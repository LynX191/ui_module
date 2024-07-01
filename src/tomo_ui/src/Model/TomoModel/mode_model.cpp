#include "mode_model.h"

ModeModel::ModeModel(QObject* parent) : QAbstractListModel(parent)
{
}

ModeModel::~ModeModel()
{
}

void ModeModel::addData(const ModeFeature& entry)
{
	beginInsertRows(QModelIndex(), rowCount(), rowCount());
	_model << entry;
	endInsertRows();
}

int ModeModel::rowCount(const QModelIndex& parent) const
{
	parent.isValid();
	return _model.count();
}

QVariant ModeModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid())
		return QVariant();
	ModeFeature modelEntry = _model[index.row()];
	if(role == cmdRole)
		return modelEntry.cmd;
	else if(role == aliasRole)
		return modelEntry.alias_name;
	else if(role == valuesRole)
		return modelEntry.values_name;
	else if(role == isCheckRole)
		return modelEntry.isCheck;
	return QVariant();
}

QHash<int, QByteArray> ModeModel::roleNames() const
{
	QHash<int, QByteArray> roles;
	roles[cmdRole]	   = "cmd";
	roles[aliasRole]   = "alias_name";
	roles[valuesRole]  = "values_name";
	roles[isCheckRole] = "isCheck";
	return roles;
}

void ModeModel::editInputData(int row, const QVariant& value, int role)
{
	QModelIndex index = this->index(row, 0);
	if(!index.isValid() || index.row() < 0 || index.row() >= _model.count())
		return;
	if(role == cmdRole)
		_model[index.row()].cmd = value.toString();
	else if(role == aliasRole)
		_model[index.row()].alias_name = value.toString();
	else if(role == valuesRole)
		_model[index.row()].values_name = value.toString();
	else if(role == isCheckRole)
		_model[index.row()].isCheck = value.toBool();
	Q_EMIT dataChanged(index, index, {role});
}

void ModeModel::createData(QVector<ModeFeature> data)
{
	for(int i = 0; i < data.size(); i++) {
		ModeFeature modeFeature;
		modeFeature.cmd			= data[i].cmd;
		modeFeature.alias_name	= data[i].alias_name;
		modeFeature.values_name = data[i].values_name;
		modeFeature.isCheck		= data[i].isCheck;
		_model << modeFeature;
	}
}
