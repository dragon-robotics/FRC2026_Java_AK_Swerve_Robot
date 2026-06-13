package frc.robot.subsystems.vision;

import java.util.Comparator;
import java.util.List;
import java.util.Locale;

final class VisionStrategyComparisonSupport {

  private VisionStrategyComparisonSupport() {}

  record StrategySummary(
      String strategyName,
      double maxOdomJumpMeters,
      int catastrophicOutlierCount,
      double visionCoverageRatio,
      double maxVisionDeviationMeters,
      double meanVisionDeviationMeters) {

    boolean passesHardGates(double maxJumpMeters, int maxOutliers, double minCoverageRatio) {
      return maxOdomJumpMeters <= maxJumpMeters
          && catastrophicOutlierCount <= maxOutliers
          && visionCoverageRatio >= minCoverageRatio;
    }
  }

  record RankedStrategy(StrategySummary summary, boolean passesHardGates) {
    String strategyName() {
      return summary.strategyName();
    }
  }

  record ScenarioResult(
      String scenarioName,
      List<StrategySummary> strategySummaries,
      List<RankedStrategy> rankedStrategies) {

    String bestStrategy() {
      return rankedStrategies().isEmpty() ? "NONE" : rankedStrategies().get(0).strategyName();
    }
  }

  record FullBakeoffResult(List<ScenarioResult> scenarioResults, List<String> offlineHybridRules) {}

  static List<RankedStrategy> rankStrategies(
      List<StrategySummary> summaries,
      double maxJumpMeters,
      int maxOutliers,
      double minCoverageRatio) {
    return summaries.stream()
        .map(
            summary ->
                new RankedStrategy(
                    summary, summary.passesHardGates(maxJumpMeters, maxOutliers, minCoverageRatio)))
        .sorted(
            Comparator.comparing(RankedStrategy::passesHardGates)
                .reversed()
                .thenComparing(strategy -> strategy.summary.meanVisionDeviationMeters())
                .thenComparing(strategy -> strategy.summary.maxVisionDeviationMeters())
                .thenComparing(
                    strategy -> strategy.summary.visionCoverageRatio(), Comparator.reverseOrder()))
        .toList();
  }

  static List<String> deriveOfflineHybridRules(List<ScenarioResult> results) {
    boolean constrainedWinsAny =
        results.stream().anyMatch(result -> result.bestStrategy().equals("CONSTRAINED_SOLVEPNP"));
    boolean multitagWinsAny =
        results.stream()
            .anyMatch(result -> result.bestStrategy().equals("MULTI_TAG_PNP_ON_COPROCESSOR"));
    boolean trigWinsAny =
        results.stream()
            .anyMatch(result -> result.bestStrategy().equals("PNP_DISTANCE_TRIG_SOLVE"));

    var rules = new java.util.ArrayList<String>();
    if (constrainedWinsAny) {
      rules.add(
          "if angularRateRadPerSec is high or excess pose jump risk increases, prefer CONSTRAINED_SOLVEPNP");
    }
    if (multitagWinsAny) {
      rules.add(
          "if strong multi-tag visibility passes hard gates with lower deviation, prefer MULTI_TAG_PNP_ON_COPROCESSOR");
    }
    if (trigWinsAny) {
      rules.add(
          "if heading-conditioned solvers are unavailable and trig remains within hard gates, prefer PNP_DISTANCE_TRIG_SOLVE");
    }
    if (rules.isEmpty()) {
      rules.add(
          "no stable hybrid recommendation could be derived from the current bakeoff results");
    }
    return rules;
  }

  static String formatSummaryLine(String prefix, String scenarioName, StrategySummary summary) {
    return String.format(
        Locale.US,
        "%s|%s|%s maxExcessJump=%.4f m outliers=%d coverage=%.3f maxVisDev=%.4f m meanVisDev=%.4f m",
        prefix,
        scenarioName,
        summary.strategyName(),
        summary.maxOdomJumpMeters(),
        summary.catastrophicOutlierCount(),
        summary.visionCoverageRatio(),
        summary.maxVisionDeviationMeters(),
        summary.meanVisionDeviationMeters());
  }
}
