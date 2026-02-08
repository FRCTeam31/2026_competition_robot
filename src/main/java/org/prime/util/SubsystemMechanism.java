package org.prime.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import javax.management.openmbean.KeyAlreadyExistsException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * @deprecated To be removed next commit, will use {@code LoggedMechanism2d.generate3dMechanism()} instead.
 * This is present in the commit to document this approach, {@code generate3dMechanism()} will work better
 * for our use case.
 */
@Deprecated
public class SubsystemMechanism {
    private LoggedMechanism2d _mechanism;
    private LoggedMechanismRoot2d _root;
    private final Map<String, LoggedMechanismLigament2d> _ligaments = new HashMap<>();
    private final Map<String, Double> _ligamentLengths = new HashMap<>();
    private final Map<String, Pose3d> _poses = new HashMap<>();
    private final Map<String, Supplier<Translation3d>> _translationSuppliers = new HashMap<>();
    private final Map<String, Supplier<Rotation3d>> _rotationSuppliers = new HashMap<>();
    private final Supplier<Pose3d> _robotPoseSupplier;

    public SubsystemMechanism(Supplier<Pose3d> robotPoseSupplier) {
        _robotPoseSupplier = robotPoseSupplier;
    }

    public void updateMechanism() {
        _ligaments.forEach((key, ligament) -> {
            ligament.setAngle(_rotationSuppliers.get(key).get().toRotation2d());
            ligament.setLength(_translationSuppliers.get(key).get().getNorm() + _ligamentLengths.get(key));
        });
    }

    public void createMechanism(double width, double height, Color8Bit backgroundColor) {
        _mechanism = new LoggedMechanism2d(width, height, backgroundColor);
    }

    public void createMechanism(double width, double height) {
        createMechanism(width, height, new Color8Bit(0, 0, 32));
    }

    public void createRoot(String name, double x, double y) {
        _root = _mechanism.getRoot(name, x, y);
    }

    public void appendLigament(
            String key,
            double length,
            double angle,
            double lineWidth,
            Color8Bit color,
            Supplier<Translation3d> translation3dSupplier,
            Supplier<Rotation3d> rotation3dSupplier
    ) {
        LoggedMechanismLigament2d ligament = new LoggedMechanismLigament2d(key, length, angle, lineWidth, color);

        if (_ligaments.containsKey(key)) {
            throw new KeyAlreadyExistsException();
        }

        _root.append(ligament);
        _ligaments.put(key, ligament);
        _ligamentLengths.put(key, length);
        _translationSuppliers.put(key, translation3dSupplier);
        _rotationSuppliers.put(key, rotation3dSupplier);
        _poses.put(key, new Pose3d());
    }

    public void appendLigament(
            String key,
            double length,
            double angle,
            Supplier<Translation3d> translation3dSupplier,
            Supplier<Rotation3d> rotation3dSupplier
    ) {
        appendLigament(
                key,
                length,
                angle,
                10,
                new Color8Bit(235, 137, 52),
                translation3dSupplier,
                rotation3dSupplier
        );
    }

    public LoggedMechanism2d getMechanism() {
        return _mechanism;
    }

    public LoggedMechanismRoot2d getRoot() {
        return _root;
    }

    public List<LoggedMechanismLigament2d> getLigaments() {
        return _ligaments.values().stream().toList();
    }

    public LoggedMechanismLigament2d getLigament(String key) {
        return _ligaments.get(key);
    }

    public List<Pose3d> getRobotRelativePoses() {
        return _poses.entrySet().stream().map(entry -> {
            String key = entry.getKey();
            Pose3d pose = entry.getValue();

            return pose.transformBy(new Transform3d(
                    _translationSuppliers.get(key).get(),
                    _rotationSuppliers.get(key).get()
            ));
        }).toList();
    }

    public Pose3d getRobotRelativePose(String key) {
        return _poses.get(key).transformBy(new Transform3d(
                _translationSuppliers.get(key).get(),
                _rotationSuppliers.get(key).get()
        ));
    }

    public List<Pose3d> getFieldRelativePoses() {
        Pose3d robotPose = _robotPoseSupplier.get();
        return _poses.entrySet().stream().map(entry -> {
            String key = entry.getKey();
            Pose3d pose = entry.getValue();

            return pose.transformBy(new Transform3d(
                    _translationSuppliers.get(key).get(),
                    _rotationSuppliers.get(key).get()
            )).transformBy(new Transform3d(
                    robotPose.getTranslation(),
                    robotPose.getRotation()
            ));
        }).toList();
    }

    public Pose3d getFieldRelativePose(String key) {
        Pose3d robotPose = _robotPoseSupplier.get();

        return _poses.get(key).transformBy(new Transform3d(
                _translationSuppliers.get(key).get(),
                _rotationSuppliers.get(key).get()
        )).transformBy(new Transform3d(
                robotPose.getTranslation(),
                robotPose.getRotation()
        ));
    }
}
