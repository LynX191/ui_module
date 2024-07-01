#ifndef LOGINMODEL_H
#define LOGINMODEL_H

#include <QAbstractListModel>

// My include
#include "../ViewModel/login_vm.h"
// end

class LoginVM;

class LoginModel : public QAbstractListModel
{
	Q_OBJECT
	Q_PROPERTY(LoginVM* list READ list WRITE setList)

public:
	explicit LoginModel(QObject* parent = nullptr);
	~LoginModel();

	enum
	{
		UserRole = Qt::UserRole,
		LevelRole,
		CheckRole
	};
	// Basic functionality:
	int rowCount(const QModelIndex& parent = QModelIndex()) const override;

	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

	// Editable:
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;

	Qt::ItemFlags flags(const QModelIndex& index) const override;

	virtual QHash<int, QByteArray> roleNames() const override;

	LoginVM* list() const;
	void setList(LoginVM* list);

private:
	LoginVM* mList;
};

#endif	// LOGINMODEL_H
