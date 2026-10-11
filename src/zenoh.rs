//! zenoh glue without a zenoh dependency: `zenoh_recorder!(zenoh)` expands against the caller's own zenoh crate, so one
//! rust-recorder works with zenoh 1.6 (Desktop's gateway pin) and 1.10 (dimos' pin) alike. Cargo can't hold two
//! semver-compatible `=` pins of one crate as features, so a macro is the way to support both.

/// Defines `zenoh_recording::{record_key, record_dimos_key}` against the zenoh crate named by the argument (`zenoh`,
/// or a renamed dependency such as `zenoh16`).
///
/// ```ignore
/// rust_recorder::zenoh_recorder!(zenoh);
/// let (stream, subscriber) = zenoh_recording::record_dimos_key(&session, recorder.clone(), "dimos/odom/nav_msgs.Odometry", || true).await?;
/// ```
#[macro_export]
macro_rules! zenoh_recorder {
    ($zenoh:ident) => {
        pub mod zenoh_recording {
            use ::std::sync::Arc;

            /// Records every sample on `key` into `stream` while the subscriber lives; `admit` decides per sample
            /// (false = counted as skipped, e.g. over a max rate).
            pub async fn record_key<Admit>(
                session: &$zenoh::Session,
                recorder: Arc<$crate::Recorder>,
                stream: $crate::Stream,
                key: &str,
                admit: Admit,
            ) -> $crate::anyhow::Result<$zenoh::pubsub::Subscriber<()>>
            where
                Admit: Fn() -> bool + Send + Sync + 'static,
            {
                session
                    .declare_subscriber(key.to_string())
                    .callback(move |sample| {
                        if admit() {
                            recorder.record(&stream, &sample.payload().to_bytes());
                        } else {
                            recorder.skip(&stream);
                        }
                    })
                    .await
                    .map_err(|error| $crate::anyhow::anyhow!("subscribe {key}: {error}"))
            }

            /// Adds the stream a dimos key carries (`dimos/<port>/<pkg.Type>`, named `/<port>`) and records it.
            pub async fn record_dimos_key<Admit>(
                session: &$zenoh::Session,
                recorder: Arc<$crate::Recorder>,
                key: &str,
                admit: Admit,
            ) -> $crate::anyhow::Result<($crate::Stream, $zenoh::pubsub::Subscriber<()>)>
            where
                Admit: Fn() -> bool + Send + Sync + 'static,
            {
                let spec = $crate::StreamSpec::from_dimos_key(key).ok_or_else(|| $crate::anyhow::anyhow!("not a dimos key: {key}"))?;
                let stream = recorder.add_stream(spec)?;
                let subscriber = record_key(session, recorder, stream.clone(), key, admit).await?;
                Ok((stream, subscriber))
            }
        }
    };
}
