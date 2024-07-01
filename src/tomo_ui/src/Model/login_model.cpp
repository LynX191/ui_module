#include "login_model.h"

LoginModel::LoginModel(QObject* parent) : QAbstractListModel(parent), mList(nullptr)
{
}

LoginModel::~LoginModel()
{

}

int LoginModel::rowCount(const QModelIndex& parent) const
{
	// For list models only the root node (an invalid parent) should return the list's size. For all
	// other (valid) parents, rowCount() should return 0 so that it does not become a tree model.
	if(parent.isValid() || !mList)
		return 0;

	// FIXME: Implement me!
	return mList->items().size();
}

QVariant LoginModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid() || !mList) {
		return QVariant();
	}

	// FIXME: Implement me!
	const AccountFeature item = mList->items().at(index.row());
	switch(role) {
	case UserRole:
		return QVariant(item.user);
	case LevelRole:
		return QVariant(item.level);
	case CheckRole:
		return QVariant(item.check);
	default:
		break;
	}
	return QVariant();
}

bool LoginModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if(!mList)
		return false;

	AccountFeature item = mList->items().at(index.row());
	switch(role) {
	case UserRole:
		item.user = value.toString();
		break;
	case LevelRole:
		item.level = value.toString();
		break;
	case CheckRole:
		item.check = value.toBool();
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

Qt::ItemFlags LoginModel::flags(const QModelIndex& index) const
{

	if(!index.isValid())
		return Qt::NoItemFlags;

	return Qt::ItemIsEditable;	// FIXME: Implement me!
}

QHash<int, QByteArray> LoginModel::roleNames() const
{

	QHash<int, QByteArray> names;
	names[UserRole]	 = "username";
	names[LevelRole] = "level";
	names[CheckRole] = "checked";
	return names;
}

LoginVM* LoginModel::list() const
{
	return mList;
}

void LoginModel::setList(LoginVM* list)
{
	beginResetModel();
	if(mList)
		mList->disconnect(this);
	mList = list;
	if(mList) {
		connect(mList, &LoginVM::preAppendItem, this, [=]() {
			const int index = mList->items().size();
			beginInsertRows(QModelIndex(), index, index);
		});
		connect(mList, &LoginVM::postAppendItem, this, [=]() { endInsertRows(); });
		connect(mList, &LoginVM::preRemoveItem, this, [=](int index) { beginRemoveRows(QModelIndex(), index, index); });
		connect(mList, &LoginVM::postRemoveItem, this, [=]() { endRemoveRows(); });
	}
	endResetModel();
}
