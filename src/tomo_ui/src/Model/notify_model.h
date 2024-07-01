#ifndef NOTIFYMODEL_H
#define NOTIFYMODEL_H

#include <QAbstractListModel>

// My include
#include "../ViewModel/NotifyVM/notify_vm.h"
// end

class NotifyVM;

class NotifyModel : public QAbstractListModel
{
	Q_OBJECT
    Q_PROPERTY(NotifyVM* list READ list WRITE setList)

public:
    explicit NotifyModel(QObject* parent = nullptr);
    ~NotifyModel();

	enum
	{
        TypeRole = Qt::UserRole ,
        MessIDRole,
        ContentRole,
        TimeRole
	};
	// Basic functionality:
	int rowCount(const QModelIndex& parent = QModelIndex()) const override;

	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

	// Editable:
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;

	Qt::ItemFlags flags(const QModelIndex& index) const override;

	virtual QHash<int, QByteArray> roleNames() const override;

    NotifyVM* list() const;
    void setList(NotifyVM* list);

private:
    NotifyVM* mList;

};


#endif	// NOTIFYMODEL_H
