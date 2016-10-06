#include <QStringList>

#include <base_placement_plugin/point_tree_item.h>

PointTreeItem::PointTreeItem(const QVector< QVariant > &data, PointTreeItem *parent)
{
  parentItem = parent;
  itemData = data;
}

PointTreeItem::~PointTreeItem()
{
  qDeleteAll(childItems);
}

PointTreeItem *PointTreeItem::child(int number)
{
  return childItems.value(number);
}

int PointTreeItem::childCount() const
{
  return childItems.count();
}

int PointTreeItem::childNumber() const
{
  if (parentItem)
    return parentItem->childItems.indexOf(const_cast< PointTreeItem * >(this));

  return 0;
}

int PointTreeItem::columnCount() const
{
  return itemData.count();
}

QVariant PointTreeItem::data(int column) const
{
  return itemData.value(column);
}

bool PointTreeItem::insertChildren(int position, int count, int columns)
{
  if (position < 0 || position > childItems.size())
    return false;

  for (int row = 0; row < count; ++row)
  {
    QVector< QVariant > data(columns);
    PointTreeItem *item = new PointTreeItem(data, this);
    childItems.insert(position, item);
  }

  return true;
}

bool PointTreeItem::insertColumns(int position, int columns)
{
  if (position < 0 || position > itemData.size())
    return false;

  for (int column = 0; column < columns; ++column)
    itemData.insert(position, QVariant());

  PointTreeItem *child;
  for (int i = 0; i < childItems.count(); i++)
  {
    child = childItems.at(i);
    child->insertColumns(position, columns);
  }

  return true;
}

PointTreeItem *PointTreeItem::parent()
{
  return parentItem;
}

bool PointTreeItem::removeChildren(int position, int count)
{
  if (position < 0 || position + count > childItems.size())
    return false;

  for (int row = 0; row < count; ++row)
    delete childItems.takeAt(position);

  return true;
}

bool PointTreeItem::removeColumns(int position, int columns)
{
  if (position < 0 || position + columns > itemData.size())
    return false;

  for (int column = 0; column < columns; ++column)
    itemData.remove(position);

  PointTreeItem *child;
  for (int i = 0; i < childItems.count(); i++)
  {
    child = childItems.at(i);
    child->removeColumns(position, columns);
  }

  return true;
}

bool PointTreeItem::setData(int column, const QVariant &value)
{
  if (column < 0 || column >= itemData.size())
    return false;

  itemData[column] = value;
  return true;
}
