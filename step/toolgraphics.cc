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

#include "toolgraphics.h"
#include "toolgraphics.moc"

#include "worldmodel.h"
#include "worldfactory.h"
#include <QItemSelectionModel>
#include <QGraphicsSceneMouseEvent>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsView>
#include <QEvent>
#include <QPainter>
#include <KLocale>

bool NoteCreator::sceneEvent(QEvent* event)
{
    if(event->type() == QEvent::GraphicsSceneMousePress) {
        _worldModel->simulationPause();
        QGraphicsSceneMouseEvent* mouseEvent = static_cast<QGraphicsSceneMouseEvent*>(event);
        QPointF pos = mouseEvent->scenePos();
        QVariant vpos = QVariant::fromValue(WorldGraphicsItem::pointToVector(pos));
        _worldModel->beginMacro(i18n("Create %1", _className));
        _item = _worldModel->newItem(_className); Q_ASSERT(_item != NULL);
        _worldModel->setProperty(_item, _item->metaObject()->property("position"), vpos);
        _worldModel->selectionModel()->setCurrentIndex(_worldModel->objectIndex(_item),
                              QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Current);
        _worldModel->endMacro();
        event->accept();
        return true;
    }
    return false;
}

NoteTextItem::NoteTextItem(NoteGraphicsItem* noteItem, QGraphicsItem* parent)
    : QGraphicsTextItem(parent), _noteItem(noteItem)
{
    setPlainText(emptyNotice());
}

QString NoteTextItem::emptyNotice() const
{
    return i18n("Click to enter a text");
}

void NoteTextItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QStyleOptionGraphicsItem opt = *option; // XXX: are there any documented way to do this ?
    if(_noteItem->isSelected()) opt.state |=  QStyle::State_HasFocus;
    QGraphicsTextItem::paint(painter, &opt, widget);
}

void NoteTextItem::focusInEvent(QFocusEvent *event)
{
    if(_noteItem->note()->text().isEmpty()) {
        _noteItem->_updating = true;
        setPlainText("");
        _noteItem->advance(1);
        _noteItem->_updating = false;
    }
    QGraphicsTextItem::focusInEvent(event);
}

void NoteTextItem::focusOutEvent(QFocusEvent *event)
{
    if(_noteItem->note()->text().isEmpty()) {
        _noteItem->_updating = true;
        setPlainText(emptyNotice());
        _noteItem->advance(1);
        _noteItem->_updating = false;
    }
    QGraphicsTextItem::focusOutEvent(event);
}

NoteGraphicsItem::NoteGraphicsItem(StepCore::Item* item, WorldModel* worldModel)
    : WorldGraphicsItem(item, worldModel)
{
    Q_ASSERT(dynamic_cast<StepCore::Note*>(_item) != NULL);
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemIsMovable);
    setAcceptsHoverEvents(true);

    _textItem = new NoteTextItem(this, this);
    _textItem->setTextInteractionFlags(Qt::TextEditorInteraction);
    _textItem->scale(1, -1);
    _lastScale = 1;
    _updating = false;
    connect(_textItem->document(), SIGNAL(contentsChanged()), this, SLOT(contentsChanged()));
    advance(1);
}

inline StepCore::Note* NoteGraphicsItem::note() const
{
    return static_cast<StepCore::Note*>(_item);
}

QPainterPath NoteGraphicsItem::shape() const
{
    QPainterPath path;
    path.addRect(_boundingRect);
    return path;
}

void NoteGraphicsItem::paint(QPainter* /*painter*/, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
    /*painter->setPen(QPen(Qt::black, 0));
    painter->setBrush(QBrush(Qt::lightGray));
    QRectF rect = boundingRect();
    painter->drawRect(rect);*/
    /*
    double s = currentViewScale();
    painter->setPen(QPen(Qt::gray, 0));
    painter->drawLine(QLineF(0, -4/s, rect.width(), -4/s)); 
    painter->drawLine(QLineF(0, -8/s, rect.width(), -8/s)); 
    */

    /*
    double s = currentViewScale();
    double radius = 6/s;

    int renderHints = painter->renderHints();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(QPen(Qt::black, 0));
    painter->setBrush(QBrush(Qt::black));
    
    painter->drawEllipse(QRectF(-radius,-radius,radius*2,radius*2));
    painter->setBrush(QBrush());
    painter->setRenderHint(QPainter::Antialiasing, renderHints & QPainter::Antialiasing);

    if(isSelected()) {
        painter->setRenderHint(QPainter::Antialiasing, true);
        painter->setPen(QPen(SELECTION_COLOR, 0, Qt::DashLine));
        //painter->setBrush(QBrush(QColor(0, 0x99, 0xff)));
        radius = (6+SELECTION_MARGIN)/s;
        painter->drawEllipse(QRectF(-radius, -radius, radius*2, radius*2));
        painter->setRenderHint(QPainter::Antialiasing, renderHints & QPainter::Antialiasing);
    }*/
}

void NoteGraphicsItem::advance(int phase)
{
    if(phase == 0) return;

    _worldModel->simulationPause();
    if(!_updating && _textItem->toPlainText() != note()->text()) {
        _updating = true;
        if(!_textItem->hasFocus() && note()->text().isEmpty()) {
            _textItem->setPlainText(_textItem->emptyNotice());
        } else {
            _textItem->setPlainText(note()->text());
        }
        _updating = false;
    }

    double s = currentViewScale();
    if(s != _lastScale) {
        _textItem->resetTransform();
        _textItem->scale(1/s, -1/s);
        _lastScale = s;
    }
    
    QPointF r = vectorToPoint(note()->position());
    QSizeF  size = _textItem->boundingRect().size()/s;
    size.setHeight(-size.height());

    if(size != _boundingRect.size()) {
        prepareGeometryChange();
        _boundingRect.setSize(size);
    }
    
    if(r != pos()) setPos(r);
    update(); // XXX: documentation says this is unnessesary, but it doesn't work without it
}

void NoteGraphicsItem::contentsChanged()
{
    if(!_updating) {
        _updating = true;
        _worldModel->simulationPause();
        _worldModel->setProperty(_item, _item->metaObject()->property("text"),
                                QVariant::fromValue( _textItem->toPlainText() ));
        _updating = false;
    }
}

void NoteGraphicsItem::mouseSetPos(const QPointF& pos, const QPointF& /*diff*/)
{
    _worldModel->simulationPause();
    _worldModel->setProperty(_item, _item->metaObject()->property("position"),
                                QVariant::fromValue( pointToVector(pos) ));
}

QVariant NoteGraphicsItem::itemChange(GraphicsItemChange change, const QVariant& value)
{
    /*
    if(change == QGraphicsItem::ItemSelectedChange && scene()) {
        if(value.toBool()) {
            _velocityHandler->setVisible(true);
        } else {
            _velocityHandler->setVisible(false);
        }
    }*/
    return WorldGraphicsItem::itemChange(change, value);
}

