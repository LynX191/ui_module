#ifndef MODEMODEL_H
#define MODEMODEL_H

#include <QAbstractListModel>


struct ModeFeature{
	QString cmd;
    QString alias_name;
    QString values_name;
    bool isCheck;
};

class ModeModel:  public QAbstractListModel
{
    Q_OBJECT

public:
    explicit ModeModel(QObject *parent = nullptr);
    ~ModeModel();

	enum DataRoles
	{
		cmdRole		= Qt::UserRole,
        aliasRole	= Qt::UserRole + 1,
        valuesRole  = Qt::UserRole + 2,
        isCheckRole = Qt::UserRole + 3
	};
	// Basic functionality:
    void addData(const ModeFeature& entry);
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override ;

    // Editable:
	void editInputData(int row, const QVariant& value, int role = Qt::EditRole);
	void createData(QVector<ModeFeature> data);


    QVector<ModeFeature> _model;
};

#endif // MODEMODEL_H
