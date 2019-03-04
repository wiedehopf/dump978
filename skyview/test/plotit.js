"use strict";

var MarkerLayer;
var NextLon = 0;
var NextLat = 0;

function setup() {
        MarkerLayer = new ol.layer.Vector({
                source: new ol.source.Vector(),
        });

        var map = new ol.Map({
                target: 'map_canvas',
                layers: [
                        new ol.layer.Tile({
                                source: new ol.source.OSM(),
                                name: 'osm',
                                title: 'OpenStreetMap',
                                type: 'base',
                        }),
                        MarkerLayer
                ],
                view: new ol.View({
                        center: ol.proj.fromLonLat([5, 0]),
                        zoom: 7
                }),
                controls: [new ol.control.Zoom(),
                           new ol.control.Rotate(),
                           new ol.control.ScaleLine({units: "metric"})],
                loadTilesWhileAnimating: true,
                loadTilesWhileInteracting: true
        });

        var data = {
                2162: [29.1551612249733, -95.0173217092621, "dbaker"],
                2512: [29.7332515409173, -95.4344386600494, "jsulak"],
                4205: [29.7559384696116,  -95.411956555603, "dbaker"],
                5993: [29.7330970688128, -95.4345774650574, "karl"],
                7151: [29.781319294809, -95.6388580799103, "karl"],
                7187: [30.3556307545623, -95.2642798423767, "karl"],
                13370: [29.7511111445497, -95.3980131778717, "nugget"],
                14213: [29.8061814357023, -95.5617366763347, "cbw"],
                14408: [29.7330379101277, -95.4344265460967, "dbaker"],
                20170: [      29.7502326,        -95.382848, "ericcarlson"],
                24294: [          29.702,           -95.526, "lkowolowksi"],
                25611: [       29.733032,          -95.4344, "ericcarlson"],
                27732: [29.7331515041002, -95.4346116428375, "lkowolowksi"],
                27840: [29.7534129680077, -95.6198360919952, "michael179"],
                28243: [       29.733032,          -95.4344, "ericcarlson"],
                30139: [       28.805038,        -95.658935, "ashleyguinard"],
        };

        for (var siteid in data) {
                var lat = data[siteid][0];
                var lon = data[siteid][1];
                var marker = new ol.Feature(new ol.geom.Point(ol.proj.fromLonLat([lon, lat])));
                marker.setStyle(new ol.style.Style({
                        image: new ol.style.Circle({
                                radius: 7,
                                snapToPixel: false,
                                fill: new ol.style.Fill({color: 'black'}),
                                stroke: new ol.style.Stroke({
                                        color: 'white', width: 2
                                })
                        }),

                        text: new ol.style.Text({
                                offsetY: 12,
                                text: siteid + ": " + data[siteid][2]
                        })
                }));

                MarkerLayer.getSource().addFeature(marker);
        }
}
