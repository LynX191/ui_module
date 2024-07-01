#include "state_model.h"

StateModel::StateModel(QObject* parent) : QAbstractListModel(parent)
{
}

StateModel::~StateModel()
{
}

void StateModel::addData(const StateFeature& entry)
{
	beginInsertRows(QModelIndex(), rowCount(), rowCount());
	_model << entry;
	endInsertRows();
}

int StateModel::rowCount(const QModelIndex& parent) const
{
	parent.isValid();
	return _model.count();
}

QVariant StateModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid())
		return QVariant();
	StateFeature modelEntry = _model[index.row()];
	if(role == nameRole) {
		return modelEntry.name;
	}
	if(role == isCheckRole) {
		return modelEntry.isCheck;
	}
	return QVariant();
}

QHash<int, QByteArray> StateModel::roleNames() const
{
	QHash<int, QByteArray> roles;
	roles[nameRole]	   = "name";
	roles[isCheckRole] = "isCheck";
	return roles;
}

void StateModel::editInputData(int row, const QVariant& value, int role)
{
	QModelIndex index = this->index(row, 0);
	if(!index.isValid() || index.row() < 0 || index.row() >= _model.count())
		return;
	if(role == nameRole) {
		_model[index.row()].name = value.toString();
	}
	if(role == isCheckRole) {
		_model[index.row()].isCheck = value.toBool();
	}
	Q_EMIT dataChanged(index, index, {role});
}

void StateModel::createData(QVector<StateFeature> data)
{
	for(int i = 0; i < data.size(); i++) {
		StateFeature stateFeature;
		stateFeature.name	 = data[i].name;
		stateFeature.isCheck = data[i].isCheck;
		_model << stateFeature;
	}
}
