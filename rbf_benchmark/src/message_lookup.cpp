//
// Created by Stefan Fabian on 18.09.19.
//

#include <benchmark/benchmark.h>
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
#include <ros_babel_fish/generation/providers/embedded_python_description_provider.h>


static void EmbeddedPythonDescriptionProviderCreate( benchmark::State &state )
{
  for ( auto _ : state )
  {
    ros_babel_fish::EmbeddedPythonDescriptionProvider provider;
  }
}

BENCHMARK( EmbeddedPythonDescriptionProviderCreate )->Unit( benchmark::kMillisecond );

static void EmbeddedPythonDescriptionProviderLookUp( benchmark::State &state )
{
  for ( auto _ : state )
  {
    state.PauseTiming();
    ros_babel_fish::EmbeddedPythonDescriptionProvider provider;
    state.ResumeTiming();
    provider.getMessageDescription( "ros_babel_fish_test_msgs/TestArray" );
  }
}

BENCHMARK( EmbeddedPythonDescriptionProviderLookUp )->Unit( benchmark::kMicrosecond );

static void IntegratedDescriptionProviderCreate( benchmark::State &state )
{
  for ( auto _ : state )
  {
    ros_babel_fish::IntegratedDescriptionProvider provider;
  }
}

BENCHMARK( IntegratedDescriptionProviderCreate )->Unit( benchmark::kMillisecond );

static void IntegratedDescriptionProviderLookUp( benchmark::State &state )
{
  for ( auto _ : state )
  {
    state.PauseTiming();
    ros_babel_fish::IntegratedDescriptionProvider provider;
    state.ResumeTiming();
    provider.getMessageDescription( "ros_babel_fish_test_msgs/TestArray" );
  }
}

BENCHMARK( IntegratedDescriptionProviderLookUp )->Unit( benchmark::kMicrosecond );


BENCHMARK_MAIN();