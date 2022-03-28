using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using XCharts.Runtime;
using System.Threading;
using UnityEngine.UI;


public class LineChart1Manager : MonoBehaviour
{
    LineChart chart;
    string lineChart_title = "示波器";
    bool lineChart_tooltip = true;
    bool lineChart_legend = true;
    int maxCacheDataNumber = 2000;
    bool insertDataToHead = false;

    

    void Awake()
    {
        chart = gameObject.GetComponent<LineChart>();
        if (chart == null)
        {
            chart = gameObject.AddComponent<LineChart>();
            chart.Init();
        }

        chart.SetSize(500, 200);

        var title = chart.GetOrAddChartComponent<Title>();
        title.text = lineChart_title;
        title.textStyle.fontSize = 14;

        var tooltip = chart.GetOrAddChartComponent<Tooltip>();
        tooltip.show = lineChart_tooltip;

        var legend = chart.GetOrAddChartComponent<Legend>();
        legend.show = lineChart_legend;
        legend.textStyle.fontSize = 14;

        var xAxis = chart.GetOrAddChartComponent<XAxis>();
        xAxis.splitNumber = 0;
        xAxis.boundaryGap = true;
        xAxis.type = Axis.AxisType.Category;
        xAxis.minMaxType = Axis.AxisMinMaxType.MinMax;
        xAxis.maxCache = maxCacheDataNumber;
        xAxis.insertDataToHead = insertDataToHead;

        var yAxis = chart.GetOrAddChartComponent<YAxis>();
        yAxis.type = Axis.AxisType.Value;
        yAxis.minMaxType = Axis.AxisMinMaxType.MinMax;


        var dataZoom = chart.GetOrAddChartComponent<DataZoom>();
        dataZoom.filterMode = DataZoom.FilterMode.Filter;

        chart.RemoveData();

        //添加一条数据线
        chart.AddSerie<Line>("左电机速度");
        var serie = chart.GetSerie<Line>(0);
        serie.symbol.show = false;
        serie.animation.enable = false;
        serie.lineStyle.width = 1;

        serie.maxCache = maxCacheDataNumber;
        serie.insertDataToHead = insertDataToHead;

        //添加一条数据线
        chart.AddSerie<Line>("右电机速度");
        serie = chart.GetSerie<Line>(1);
        serie.symbol.show = false;
        serie.animation.enable = false;
        serie.lineStyle.width = 1;

        serie.maxCache = maxCacheDataNumber;
        serie.insertDataToHead = insertDataToHead;

        //添加一条数据线
        chart.AddSerie<Line>("舵机转向/10");
        serie = chart.GetSerie<Line>(2);
        serie.symbol.show = false;
        serie.animation.enable = false;
        serie.lineStyle.width = 1;

        serie.maxCache = maxCacheDataNumber;
        serie.insertDataToHead = insertDataToHead;

        //添加一条数据线
        chart.AddSerie<Line>("转向误差/100");
        serie = chart.GetSerie<Line>(3);
        serie.symbol.show = false;
        serie.animation.enable = false;
        serie.lineStyle.width = 1;

        serie.maxCache = maxCacheDataNumber;
        serie.insertDataToHead = insertDataToHead;


    }

    // Update is called once per frame
    void Update()
    {
        
        chart.RefreshChart();
        chart.RefreshDataZoom();

    }
}
