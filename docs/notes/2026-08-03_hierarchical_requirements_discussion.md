# Notes: Hierarchical requirements and AI-overlooked errors

Conversation transcript, 2026-08-03 (Cursor agent session, plan/discussion mode).
Kept as raw material for a future article. This directory is excluded from AI
context via `.cursorignore` but tracked in git.

---

## User (04:24)

This is just a general discussion topic. No project changes.

I find that the errors which are overlooked by the AI are those which would
occur without a high level view of the project's objectives. Even with detailed
requirements, it seems that a higher level functional requirement is needed by
which the detailed requirements can be measured.

Would this help avoid errors where something is overlooked because it is
missing from the detailed requirements, but is clearly needed to achieve the
project high level objective?

Can the AI process requirements in a heirarchical manner this way?

## Assistant

Yes, on both counts — and this matches how the failure mode actually arises.
Let me unpack it.

**Why detailed requirements alone fail.** A detailed requirement list is always
a lossy projection of the real objective. Whoever wrote it (human or AI) made
judgment calls about what was "obvious enough" to leave out. A human
implementer silently re-derives the missing pieces because they hold the
objective in their head; an AI working strictly from the detail list has no
basis for that re-derivation. The error you describe — something missing from
the details but clearly needed for the goal — is precisely a case where the
detail list underdetermines the objective and there's no stated objective to
fall back on. Your own repo has a concrete example: the inter-episode
target-clear bug. Every detailed step ("spawn episode N's targets", "remove
episode N's target ids") was satisfied, but the higher-level invariant — *each
episode plans from a clear field* — was violated because candidate ids were
disjoint across episodes. Once that invariant was written down explicitly
(it's now in `.cursor/rules/35-isaac-smoke-physx-and-logs.mdc`), the class of
bug becomes checkable rather than inferable.

**Can AI process requirements hierarchically? Yes, but only if the hierarchy is
in context.** Models don't have a persistent "project understanding" they carry
between judgments; they reason over what's in front of them. So the practical
answer is: the hierarchy must be an artifact, not an assumption. Three things
make it work well:

1. **State objectives and invariants as first-class text, separately from
   procedures.** "The suite must fail if any prohibited contact occurs" is an
   invariant; "subscribe to PhysX contact reports" is a mechanism. When both
   are present, the AI can notice when a proposed change satisfies the
   mechanism but breaks the invariant (e.g., a refactor that keeps the
   subscription but stops checking the result). When only the mechanism is
   stated, that check is impossible.

2. **Make detailed requirements traceable to their parent.** A detail like
   "clear all prims under `/World/Phase7_2/Targets`" is much more robust when
   annotated with *why* ("so each episode plans from a clear field"). The
   "why" lets the AI generalize correctly when the situation shifts slightly —
   a new replay path, a new spawn mechanism — instead of applying the letter
   of the rule in a context where it no longer achieves the intent.

3. **Ask for the check explicitly.** The most reliable pattern is to make
   objective-conformance a step, not a hope: "before finishing, verify the
   change against the invariants in spec section X" or acceptance criteria
   phrased as observable outcomes ("no episode starts with residual targets
   present") rather than implementation steps. AI is quite good at auditing a
   change against a stated invariant; it's much weaker at spontaneously
   recalling an unstated one.

**The caveat.** Hierarchy doesn't eliminate the problem, it moves it up a
level. The high-level objective is itself a lossy statement of intent, and
there will be cases where the details and the objective are both satisfied and
the outcome is still wrong. But the error rate falls a lot, because most
"overlooked" errors live exactly in the gap between mechanism and intent — and
that gap is what the hierarchy fills. There's also a context-budget trade-off:
a huge objectives document that's always attached dilutes attention. The
pattern you've converged on in this repo is close to the sweet spot: a small
set of always-applied invariants (fail-closed rules, "never weaken
thresholds"), a `spec.md` consulted for architectural changes, and per-phase
acceptance criteria. The one refinement I'd suggest is consistently writing
each detailed requirement with its parent objective inline ("do X *so that*
Y"), because that's the form that lets an AI catch the "X was done but Y still
fails" cases — which is the exact class of error you're describing.

## User (04:38)

I understand your point about the gap between intent and mechanism, and I
agree with your point ragarding the need to avoid diluting attention. Is there
a way to create detailed requirements, separately from the parent objective?
In other words, "do X, reference objective Y"? Since the objectives will
change less often it and will be referenced repeatedly by each detailed
requirement, so this would be structurally more appealing.

## Assistant

Yes — what you're describing is essentially requirements traceability with a
normative objectives registry, and it's structurally sound. It's how
safety-critical standards (DO-178C, ISO 26262) organize things: a small,
stable set of high-level objectives with stable IDs, and every detailed
requirement citing the ID of the objective it serves. It works well with AI
too, with one important adaptation.

**The structure.** Keep a single registry — one page, not more — where each
objective is a stable ID plus a one-to-two-line normative statement:

```
OBJ-CLEAR-FIELD: Every episode's target field is planned from an empty
  /World/Phase7_2/Targets subtree; no residue survives across episodes or
  replay passes.
OBJ-FAIL-CLOSED: Any prohibited body-target contact fails the episode;
  absence of contact evidence when contact occurred is itself a defect.
```

Detailed requirements then read "Before spawning episode targets, remove all
prims under `/World/Phase7_2/Targets` [OBJ-CLEAR-FIELD]". The registry changes
rarely; the details churn freely; the linkage survives the churn.

**The adaptation AI needs: references must be dereferenceable at reasoning
time.** A human reads "[OBJ-CLEAR-FIELD]" and lazily looks it up if unsure. An
AI can only benefit from the reference if the referenced text is actually in
context when the judgment happens. So the registry must either be small enough
to always attach (an always-applied rule is the natural home in Cursor —
`35-isaac-smoke-physx-and-logs.mdc` is already halfway to being this), or
there must be a standing instruction to read the registry file before changing
anything that cites an objective. If the ID resolves to text the model hasn't
loaded, it degrades into an opaque token that gets pattern-matched past, and
you lose the whole benefit.

Two refinements that make the reference scheme work harder for you:

1. **Make the IDs self-describing.** `OBJ-CLEAR-FIELD` carries most of its
   meaning in the name; `OBJ-017` carries none. A meaningful slug means that
   even when the registry isn't fully attended to, the reference itself still
   communicates intent. This is a cheap partial hedge against the
   dereferencing problem.

2. **You get reverse traceability for free, and it's the more powerful
   direction.** Once details cite objectives, you can ask "list every
   requirement that serves OBJ-CLEAR-FIELD" and audit coverage: is the
   invariant enforced on the replay path? On the GUI path? That query is
   exactly how the target-field bug class gets caught *before* it happens —
   the objective was covered on the spawn path but nobody had written the
   citation for the `--no-auto-exit` replay path, and the gap would have been
   visible as a missing entry rather than a silent omission.

The one place I'd still deviate slightly from pure ID-references: when a
detailed requirement's connection to its objective is non-obvious, keep a
half-line gloss at the reference site ("remove *all* prims, not just next
episode's ids — disjoint candidate ids leave residue [OBJ-CLEAR-FIELD]"). The
ID gives you structure and stability; the gloss guards against the case where
the link itself is the subtle part. Pure references are best where the
connection is mechanical; hybrid where it's the insight.

So concretely for this repo, if you ever wanted to adopt it: a short
`docs/objectives.md` (or a section at the top of `spec.md`) with slug IDs,
referenced from phase reports and the smoke rules, with the registry itself
attached as an always-applied rule. Small, stable, always in context — details
cite it rather than restating it.

## User (04:48)

Can you copy this conversation into a separate file that is not seen by the AI
for development, but is still included in the repo? I would like to use this
as notes for a future article. Perhaps this could be a separate directory,
such as "notes", whatever is the current convention.

## User (04:54)

One more thing:

Do you know of any articles on how to construct requirements documents for
optimal AI Iteraction?

## Assistant

Yes — this has become an active topic in the last couple of years under the
name "spec-driven development" (SDD), and several of the pieces map directly
onto what we discussed. Here's a reading list, roughly ordered by relevance to
your article angle.

**On writing specs for AI agents generally:**

- [How to write a good spec for AI agents](https://dev.to/addyosmani/how-to-write-a-good-spec-for-ai-agents-31i9)
  — Addy Osmani (Google Chrome). Probably the best single starting point.
  Draws on a GitHub analysis of 2,500+ real agent instruction files. Notable
  findings: the most effective specs use a *three-tier boundary system*
  (always / ask-first / never) rather than flat do-and-don't lists, one real
  code snippet beats paragraphs of style prose, and "never commit secrets" was
  the single most helpful constraint in the study. Also covers splitting specs
  by component to avoid the attention-dilution problem you raised.
- [Spec-Driven Development: The Definitive 2026 Guide](https://thebcms.com/blog/spec-driven-development)
  — a survey of the whole methodology (specify → plan → task → implement,
  with GitHub's Spec Kit tooling). Directly relevant to our conversation: its
  concept of a **"constitution"** — a small, durable, always-attached
  project-rules document that every spec and agent action must respect — is
  precisely the objectives registry we sketched. Their practical guidance also
  echoes what we converged on: keep specs 1–3 pages, review at phase
  boundaries, treat specs as durable docs that outlive the code.
- [Spec-Driven Development: From Code to Contract](https://arxiv.org/html/2602.00180v1)
  — the academic treatment (arXiv, 2026). Its "golden rule" is worth quoting
  for an article: *use the minimum level of specification rigor that removes
  ambiguity for your context*. It also formalizes the "what vs. how"
  separation — behavior-focused, testable, unambiguous — which is the
  mechanism/intent gap in different vocabulary.

**On requirement syntax and hierarchy specifically:**

- [EARS, Fifteen Years On: The Requirements Format Built for the Agent Era](https://joshmcdonald.medium.com/ears-fifteen-years-on-the-requirements-format-built-for-the-agent-era-0f78f8ff35a0)
  — EARS (Easy Approach to Requirements Syntax) is a constrained grammar from
  Rolls-Royce jet-engine requirements work ("WHEN [trigger], the [system]
  SHALL [response]"), and it's having a revival because agents parse it
  reliably — each requirement collapses to a single testable claim with no
  floating conditions. Interesting origin story for an article: Mavin
  extracted the pattern from airworthiness regulations that mixed objectives
  and requirements at different levels in prose — the exact hierarchy-collapse
  problem you described.
- [EARS Fundamentals (Specy)](https://specy.ai/reference/sysreq/ears-fundamentals/)
  — shows the traceability side in tooling form: model elements declare
  `satisfies` references back to requirement IDs, and a checker flags any
  requirement with no satisfier. That's the reverse-traceability audit ("which
  requirements serve OBJ-X, and is any path uncovered?") from our discussion,
  mechanized.
- [Living Specs for AI Agent Development](https://www.augmentcode.com/guides/living-specs-for-ai-agent-development)
  — argues the spec must be bidirectional: implementation decisions flow
  *back* into the spec, otherwise regeneration from stale requirements causes
  drift. Your repo's practice of updating `spec.md` and phase reports in the
  same change set is this pattern.

One observation for your article: the industry is converging from two
directions on the same structure. The AI-tooling world (Spec Kit, Kiro,
AGENTS.md) arrived at constitution + spec + tasks by trial and error with
agents; the safety-critical world (EARS, DO-178C-style traceability) had
objectives + requirements + verification matrices for decades. The convergence
point — a small stable objectives layer, atomic testable requirements citing
it by ID, and mechanical coverage checks — is exactly the hierarchy you
proposed, which suggests it's not incidental but forced by the same underlying
problem: any implementer without persistent global context, human or machine,
needs intent made into an artifact.
