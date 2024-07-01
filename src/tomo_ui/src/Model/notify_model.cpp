#include "notify_model.h"

NotifyModel::NotifyModel(QObject* parent) : QAbstractListModel(parent), mList(nullptr)
{
}

NotifyModel::~NotifyModel()
{

}

int NotifyModel::rowCount(const QModelIndex& parent) const
{
	// For list models only the root node (an invalid parent) should return the list's size. For all
	// other (valid) parents, rowCount() should return 0 so that it does not become a tree model.
	if(parent.isValid() || !mList)
		return 0;

	// FIXME: Implement me!
	return mList->items().size();
}

QVariant NotifyModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid() || !mList) {
		return QVariant();
	}

	// FIXME: Implement me!
    const NotifyFeature item = mList->items().at(index.row());
	switch(role) {
    case TypeRole:
        return QVariant(item.type);
    case MessIDRole:
        return QVariant(item.messID);
    case ContentRole:
        return QVariant(item.content);
    case TimeRole:
        return QVariant(item.time);
	default:
		break;
	}
	return QVariant();
}

bool NotifyModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if(!mList)
		return false;

    NotifyFeature item = mList->items().at(index.row());
	switch(role) {
    case TypeRole:
        item.type = value.toString();
		break;
    case MessIDRole:
        item.messID = value.toString();
        break;
    case ContentRole:
        item.content = value.toString();
		break;
    case TimeRole:
        item.time = value.toString();
		break;
	default:
		break;
	}
	// FIXME: Implement me!
	if(mList->setItemAt(index.row(), item)) {
		Q_EMIT dataChanged(index, index, QVector<int>() << role);
		return true;
	}
	return false;
}

Qt::ItemFlags NotifyModel::flags(const QModelIndex& index) const
{

	if(!index.isValid())
		return Qt::NoItemFlags;

	return Qt::ItemIsEditable;	// FIXME: Implement me!
}

QHash<int, QByteArray> NotifyModel::roleNames() const
{

	QHash<int, QByteArray> names;
    names[TypeRole]	 = "type";
    names[MessIDRole] = "messID";
    names[ContentRole] = "content";
    names[TimeRole] = "time";
	return names;
}

NotifyVM* NotifyModel::list() const
{
	return mList;
}

void NotifyModel::setList(NotifyVM* list)
{
    beginResetModel();
    if(mList)
        mList->disconnect(this);
    mList = list;
    if(mList) {
        connect(mList, &NotifyVM::prePrependItem, this, [=]() {
            beginInsertRows(QModelIndex(), 0, 0);  // Use index 0 for prepend
        });
        connect(mList, &NotifyVM::postPrependItem, this, [=]() { endInsertRows(); });
        connect(mList, &NotifyVM::preRemoveItem, this, [=](int index) { beginRemoveRows(QModelIndex(), index, index); });
        connect(mList, &NotifyVM::postRemoveItem, this, [=]() { endRemoveRows(); });
    }
    endResetModel();
}
