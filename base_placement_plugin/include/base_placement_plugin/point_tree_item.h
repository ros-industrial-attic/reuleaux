 #ifndef PointTreeItem_H
 #define PointTreeItem_H

 #include <QList>
 #include <QVariant>
 #include <QVector>

 class PointTreeItem
 {
 public:
     PointTreeItem(const QVector<QVariant> &data, PointTreeItem *parent = 0);
     ~PointTreeItem();

     PointTreeItem *child(int number);
     int childCount() const;
     int columnCount() const;
     QVariant data(int column) const;
     bool insertChildren(int position, int count, int columns);
     bool insertColumns(int position, int columns);
     PointTreeItem *parent();
     bool removeChildren(int position, int count);
     bool removeColumns(int position, int columns);
     int childNumber() const;
     bool setData(int column, const QVariant &value);

 private:
     QList<PointTreeItem*> childItems;
     QVector<QVariant> itemData;
     PointTreeItem *parentItem;
 };

 #endif