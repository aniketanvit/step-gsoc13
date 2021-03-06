set(step_SRCS
    arrow.cc
    clipboard.cc
    mainwindow.cc
    worldmodel.cc
    worldscene.cc
    worldgraphics.cc
    worldbrowser.cc
    propertiesbrowser.cc
    infobrowser.cc
    undobrowser.cc
    itempalette.cc
    worldfactory.cc
    particlegraphics.cc
    polygongraphics.cc
    gasgraphics.cc
    softbodygraphics.cc
    springgraphics.cc
    motorgraphics.cc
    jointgraphics.cc
    toolgraphics.cc
    simulationthread.cc
    unitscalc.cc
    latexformula.cc
    messageframe.cc
    main.cc
)

set(step_UIS
    create_gas_particles.ui
    create_softbody_items.ui
    configure_graph.ui
    configure_meter.ui
    configure_controller.ui
    configure_step_general.ui)

set(step_KCFGS
    settings.kcfgc)

if(STEPCORE_WITH_QT)
    add_definitions(-DSTEPCORE_WITH_QT)
endif(STEPCORE_WITH_QT)

if(STEPCORE_WITH_GSL)
    add_definitions(-DSTEPCORE_WITH_GSL)
endif(STEPCORE_WITH_GSL)

if(STEP_WITH_QALCULATE)
    add_definitions(-DSTEP_WITH_QALCULATE)
    add_definitions(-DSTEP_WITH_UNITSCALC)
    add_definitions(${QALCULATE_CFLAGS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${KDE4_ENABLE_EXCEPTIONS}")
    set(qalculate_LIBS ${QALCULATE_LIBRARIES})
endif(STEP_WITH_QALCULATE)

find_package(KDE4 REQUIRED)
include_directories( ${KDE4_INCLUDES} ${OPENGL_INCLUDE_DIR})
add_definitions( ${KDE4_DEFINITIONS} )

kde4_add_ui_files( step_SRCS ${step_UIS} )
kde4_add_kcfg_files( step_SRCS ${step_KCFGS} )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

kde4_add_executable(step ${step_SRCS})
target_link_libraries(step
    stepcore
    ${KDE4_KHTML_LIBS}
    ${KDE4_KNEWSTUFF3_LIBS}
    ${QT_QTOPENGL_LIBRARY}
    ${qalculate_LIBS})

install(TARGETS step ${INSTALL_TARGETS_DEFAULT_ARGS})
install(FILES step.kcfg DESTINATION  ${KCFG_INSTALL_DIR})
install(FILES stepui.rc DESTINATION  ${DATA_INSTALL_DIR}/step)
install(FILES step.knsrc DESTINATION  ${CONFIG_INSTALL_DIR})
install( PROGRAMS step.desktop DESTINATION ${XDG_APPS_INSTALL_DIR} )

add_subdirectory(icons)
add_subdirectory(data)

