/* This file is part of Step.
   Copyright (C) 2007 Vladimir Kuznetsov <ks.vladimir@gmail.com>

   Step is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   Step is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Step; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "worldscene.h"
#include "worldscene.moc"

#include "worldmodel.h"
#include "worldfactory.h"
#include "worldgraphics.h"

#include <stepcore/world.h>

#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>
#include <QItemSelectionModel>
#include <QPainter>
#include <QAction>


#include <QDebug>

class WorldSceneAxes: public QGraphicsItem
{
public:
    WorldSceneAxes(QGraphicsItem* parent = 0, QGraphicsScene* scene = 0);
    QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void advance(int phase);
protected:
    QRectF _boundingRect;
    double _viewScale;
    static const int LENGTH = 100;
    static const int ARROW_STROKE = 6;
};

WorldSceneAxes::WorldSceneAxes(QGraphicsItem* parent, QGraphicsScene* scene)
    : QGraphicsItem(parent, scene), _boundingRect(-LENGTH, -LENGTH, LENGTH*2, LENGTH*2)
{
    _viewScale = 1;
    setZValue(-100);
}

QRectF WorldSceneAxes::boundingRect() const
{
    return _boundingRect;
}

QPainterPath WorldSceneAxes::shape() const
{
    return QPainterPath();
}

void WorldSceneAxes::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
    if(!scene() || scene()->views().count() == 0) return;

    painter->setPen(QPen(Qt::gray, 0));//, Qt::DotLine, Qt::SquareCap, Qt::RoundJoin));
    painter->drawLine(QLineF(0, -LENGTH, 0, LENGTH));
    painter->drawLine(QLineF(-LENGTH, 0, LENGTH, 0));

    painter->drawLine(QLineF(0, -LENGTH, -0.5*ARROW_STROKE, -LENGTH+0.866*ARROW_STROKE ));
    painter->drawLine(QLineF(0, -LENGTH, +0.5*ARROW_STROKE, -LENGTH+0.866*ARROW_STROKE ));
    painter->drawLine(QLineF(LENGTH, 0, LENGTH-0.866*ARROW_STROKE, -0.5*ARROW_STROKE ));
    painter->drawLine(QLineF(LENGTH, 0, LENGTH-0.866*ARROW_STROKE, +0.5*ARROW_STROKE ));

    painter->drawText(QRectF(5, -LENGTH, LENGTH-5, LENGTH), Qt::AlignLeft | Qt::AlignTop,
                                                QString::number( LENGTH/_viewScale ));
    painter->drawText(QRectF(5, -LENGTH, LENGTH-5, LENGTH), Qt::AlignRight | Qt::AlignBottom,
                                                QString::number( LENGTH/_viewScale ));
}

void WorldSceneAxes::advance(int phase)
{
    if(phase == 0) return;
    if(scene()) {
        prepareGeometryChange();
        _viewScale = static_cast<WorldScene*>(scene())->currentViewScale();
        resetMatrix();
        scale(1/_viewScale, -1/_viewScale);
        update(); // XXX: documentation says this is unnessesary, but it doesn't work without it
    }
}

WorldScene::WorldScene(WorldModel* worldModel, QObject* parent)
    : QGraphicsScene(parent), _worldModel(worldModel), _itemCreator(NULL), _currentViewScale(1)
{
    setItemIndexMethod(NoIndex);
    //XXX
    //setSceneRect(-200,-200,400,400);

    worldModelReset();

    QObject::connect(_worldModel, SIGNAL(modelReset()), this, SLOT(worldModelReset()));
    QObject::connect(_worldModel, SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
                         this, SLOT(worldDataChanged(const QModelIndex&, const QModelIndex&)));
    QObject::connect(_worldModel->selectionModel(), SIGNAL(currentChanged(const QModelIndex&, const QModelIndex&)),
                                           this, SLOT(worldCurrentChanged(const QModelIndex&, const QModelIndex&)));
    QObject::connect(_worldModel->selectionModel(), SIGNAL(selectionChanged(const QItemSelection&, const QItemSelection&)),
                                           this, SLOT(worldSelectionChanged(const QItemSelection&, const QItemSelection&)));
    QObject::connect(_worldModel, SIGNAL(rowsInserted(const QModelIndex&, int, int)),
                         this, SLOT(worldRowsInserted(const QModelIndex&, int, int)));
    QObject::connect(_worldModel, SIGNAL(rowsAboutToBeRemoved(const QModelIndex&, int, int)),
                         this, SLOT(worldRowsAboutToBeRemoved(const QModelIndex&, int, int)));
}

WorldScene::~WorldScene()
{
}

StepCore::Item* WorldScene::itemFromGraphics(QGraphicsItem* graphicsItem)
{
    WorldGraphicsItem* worldGraphicsItem = qgraphicsitem_cast<WorldGraphicsItem*>(graphicsItem);
    if(worldGraphicsItem != NULL) return worldGraphicsItem->item();
    else return NULL;
}

QGraphicsItem* WorldScene::graphicsFromItem(QObject* item)
{
    return _itemsHash.value(item, NULL);
}

void WorldScene::beginAddItem(const QString& name)
{
    if(_itemCreator) {
        emit endAddItem(_itemCreator->name(), false);
        delete _itemCreator;
    }
    _itemCreator = _worldModel->worldFactory()->newItemCreator(name, this, _worldModel);
    if(_itemCreator == NULL) {
        emit endAddItem(name, false);
        return;
    }
}

bool WorldScene::event(QEvent* event)
{
    if(_itemCreator) {
        if(_itemCreator->sceneEvent(event)) {
            emit endAddItem(_itemCreator->name(), _itemCreator->item() != NULL);
            delete _itemCreator;
            _itemCreator = NULL;
        }
        if(event->isAccepted()) return true;
    }
    return QGraphicsScene::event(event);
}

void WorldScene::mousePressEvent(QGraphicsSceneMouseEvent* mouseEvent)
{
    if(itemAt(mouseEvent->scenePos()) == NULL) {
        // XXX: how to easily select World ?
        //_worldModel->selectionModel()->clearSelection();
        _worldModel->selectionModel()->setCurrentIndex(_worldModel->worldIndex(), QItemSelectionModel::Clear);
    }

    QGraphicsScene::mousePressEvent(mouseEvent);
}

void WorldScene::keyPressEvent(QKeyEvent* keyEvent)
{
    if(keyEvent->matches(QKeySequence::Delete)) {
        while(!selectedItems().isEmpty()) {
            _worldModel->deleteItem(itemFromGraphics(selectedItems()[0]));
        }
        keyEvent->accept();
    } else QGraphicsScene::keyPressEvent(keyEvent);
}

/*
void WorldScene::pObjectAdded(PObject* pObject)
{
    addItem(pObject->graphicsItem());
}

void WorldScene::pObjectRemoved(PObject* pObject)
{
    removeItem(pObject->graphicsItem());
}
*/

void WorldScene::worldModelReset()
{
    /* Clear */
    while(!items().isEmpty()) {
        QGraphicsItem* item = items()[0];
        removeItem(item);
        delete item;
    }
    _itemsHash.clear();

    /* Axes */
    //new WorldSceneAxes(0, this);
    addItem(new WorldSceneAxes());
    advance();

    /* Check for new items */
    for(int i=0; i<_worldModel->itemCount(); ++i) {
        worldRowsInserted(_worldModel->worldIndex(), i, i);
    }
}

void WorldScene::worldRowsInserted(const QModelIndex& parent, int start, int end)
{
    if(parent != _worldModel->worldIndex()) return;
    for(int i=start; i<=end; ++i) {
        QModelIndex index = _worldModel->index(i, 0, parent);
        QGraphicsItem* graphicsItem = reinterpret_cast<QGraphicsItem*>(
            _worldModel->data(index, WorldModel::NewGraphicsItemRole).value<void*>());
        if(graphicsItem) {
            _itemsHash.insert(_worldModel->data(index, WorldModel::ObjectRole).value<QObject*>(), graphicsItem);
            addItem(graphicsItem);
            graphicsItem->advance(1);
        }
    }
}

void WorldScene::worldRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    if(parent != _worldModel->worldIndex()) return;
    for(int i=start; i<=end; ++i) {
        QModelIndex index = _worldModel->index(i, 0, parent);
        QGraphicsItem* graphicsItem = graphicsFromItem(_worldModel->data(index, WorldModel::ObjectRole).value<QObject*>());
        if(graphicsItem) {
            removeItem(graphicsItem);
            delete graphicsItem;
        }
    }
}

void WorldScene::worldCurrentChanged(const QModelIndex& current, const QModelIndex& /*previous*/)
{
    QGraphicsItem* graphicsItem = graphicsFromItem(_worldModel->data(current, WorldModel::ObjectRole).value<QObject*>());
    if(graphicsItem) graphicsItem->ensureVisible(0, 0, 1, 1);
}

void WorldScene::worldSelectionChanged(const QItemSelection& selected, const QItemSelection& deselected)
{
    foreach(QModelIndex index, selected.indexes()) {
        QGraphicsItem* item = _itemsHash.value(_worldModel->data(index, WorldModel::ObjectRole).value<QObject*>());
        if(item) item->setSelected(true);
    }
    foreach(QModelIndex index, deselected.indexes()) {
        QGraphicsItem* item = _itemsHash.value(_worldModel->data(index, WorldModel::ObjectRole).value<QObject*>());
        if(item) item->setSelected(false);
    }
}

void WorldScene::worldDataChanged(const QModelIndex& /*topLeft*/, const QModelIndex& /*bottomRight*/)
{
    advance(); // XXX ?
}

void WorldScene::updateViewScale()
{
    if(!views().isEmpty()) {
        _currentViewScale = views()[0]->matrix().m11();
        advance();
    }
}

WorldGraphicsView::WorldGraphicsView(WorldScene* worldScene, QWidget* parent)
    : QGraphicsView(worldScene, parent)
{
    //worldGraphicsView->setRenderHints(QPainter::Antialiasing);
    setDragMode(QGraphicsView::RubberBandDrag);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    scale(1, -1);
    setSceneRect(-SCENE_LENGTH, -SCENE_LENGTH,
                  SCENE_LENGTH*2, SCENE_LENGTH*2);
    static_cast<WorldScene*>(scene())->updateViewScale();
}

void WorldGraphicsView::zoomIn()
{
    scale(1.25, 1.25);
    double length = SCENE_LENGTH / matrix().m11();
    setSceneRect(-length, -length, length*2, length*2);
    static_cast<WorldScene*>(scene())->updateViewScale();
}

void WorldGraphicsView::zoomOut()
{
    scale(1/1.25, 1/1.25);
    double length = SCENE_LENGTH / matrix().m11();
    setSceneRect(-length, -length, length*2, length*2);
    static_cast<WorldScene*>(scene())->updateViewScale();
}

void WorldGraphicsView::fitToPage()
{
    QRectF br = static_cast<WorldScene*>(scene())->calcItemsBoundingRect();
    QRect  ws = viewport()->rect();
    double s = 0.8 * qMin( ws.width()/br.width(), ws.height()/br.height() );
    //qDebug() << "br" << br << "ws" << ws << "s" << s;
    resetMatrix();
    scale(s, -s);
    double length = SCENE_LENGTH / s;
    //qDebug() << "length" << length;
    setSceneRect(-length, -length, length*2, length*2);
    static_cast<WorldScene*>(scene())->updateViewScale();
}

